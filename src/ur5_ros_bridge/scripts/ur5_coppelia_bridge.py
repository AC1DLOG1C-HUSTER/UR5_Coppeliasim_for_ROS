#!/usr/bin/env python3
"""Subscribe to ROS trajectory commands and drive CoppeliaSim through the remote API."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import rospy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


DEFAULT_JOINT_NAMES = [f"/joint{i}" for i in range(1, 7)]


class CoppeliaBridge:
    def __init__(self):
        self.joint_names = rospy.get_param("~coppelia_joint_names", DEFAULT_JOINT_NAMES)
        self.dt = float(rospy.get_param("~dt", 0.05))
        self.gripper_signal_name = rospy.get_param("~gripper_signal_name", "RG2_open")
        self.gripper_signal_type = rospy.get_param("~gripper_signal_type", "int32")
        self.auto_start = bool(rospy.get_param("~auto_start", True))
        self.script_object_path = rospy.get_param("~script_object_path", "/UR5")

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.done_pub = rospy.Publisher("/ur5/execution_done", String, queue_size=1, latch=True)

        self._lock = threading.Lock()
        self._queue = []
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.open_sub = rospy.Subscriber("/ur5/open_path", JointTrajectory, self._path_cb, callback_args="open")
        self.close_sub = rospy.Subscriber("/ur5/close_path", JointTrajectory, self._path_cb, callback_args="close")
        self.gripper_sub = rospy.Subscriber("/ur5/gripper_cmd", Bool, self._gripper_cb)

        self.script_handle = self._resolve_script_handle()
        rospy.loginfo("Connected to CoppeliaSim script handle: %s", self.script_handle)

    def _resolve_script_handle(self):
        candidates = [self.script_object_path, "/UR5", "/UR5#0", "/UR5#0/Script"]
        last_error = None
        for candidate in candidates:
            try:
                obj_handle = self.sim.getObject(candidate)
                return self.sim.getScript(self.sim.scripttype_childscript, obj_handle)
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"Could not resolve UR5 script handle from {candidates}: {last_error}")

    def _ensure_sim_running(self):
        if self.auto_start and self.sim.getSimulationState() == self.sim.simulation_stopped:
            self.sim.startSimulation()
            time.sleep(0.5)

    def _gripper_cb(self, msg: Bool):
        if not self.gripper_signal_name:
            rospy.logwarn_throttle(5.0, "No gripper_signal_name configured; gripper commands are ignored.")
            return

        close = bool(msg.data)
        if self.gripper_signal_type == "int32":
            self.sim.setInt32Signal(self.gripper_signal_name, 0 if close else 1)
        elif self.gripper_signal_type == "bool":
            self.sim.setBoolSignal(self.gripper_signal_name, close)
        elif self.gripper_signal_type == "string":
            self.sim.setStringSignal(self.gripper_signal_name, "close" if close else "open")
        else:
            rospy.logwarn("Unsupported gripper_signal_type=%s", self.gripper_signal_type)
            return

        rospy.loginfo("Published gripper command: %s", "close" if close else "open")

    def _path_cb(self, msg: JointTrajectory, label: str):
        with self._lock:
            self._queue.append((label, msg))

    def _execute_trajectory(self, label: str, traj: JointTrajectory):
        if not traj.points:
            raise RuntimeError(f"{label} trajectory is empty")

        self._ensure_sim_running()
        path = [list(point.positions) for point in traj.points]

        try:
            self.sim.callScriptFunction("rosExecutePath", self.script_handle, path, label)
        except TypeError:
            self.sim.callScriptFunction("rosExecutePath", self.script_handle, path, str(label))

        self.done_pub.publish(String(data=f"{label}_done"))
        rospy.loginfo("Completed %s trajectory (%d waypoints)", label, len(traj.points))

    def _worker_loop(self):
        while not rospy.is_shutdown():
            item = None
            with self._lock:
                if self._queue:
                    item = self._queue.pop(0)

            if item is None:
                time.sleep(0.01)
                continue

            label, traj = item
            try:
                self._execute_trajectory(label, traj)
            except Exception as exc:
                rospy.logerr("Failed to execute %s trajectory: %s", label, exc)
                self.done_pub.publish(String(data=f"{label}_failed"))


def main():
    rospy.init_node("ur5_coppelia_bridge", anonymous=False)
    CoppeliaBridge()
    rospy.loginfo("UR5 CoppeliaSim bridge is running.")
    rospy.spin()


if __name__ == "__main__":
    main()
