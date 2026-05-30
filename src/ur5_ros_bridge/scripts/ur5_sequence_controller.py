#!/usr/bin/env python3
"""Publish the precomputed UR5 grasping sequence over ROS topics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from threading import Event

import rospy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROJECT_DIR = (
    DEFAULT_WORKSPACE_ROOT / "src" / "UR5-Path-Planning-and-Grasping-using-Coppeliasim-"
)


def load_paths(path_file: Path):
    with path_file.open("r") as f:
        return json.load(f)


def path_to_trajectory(path, dt: float, frame_id: str = "world"):
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = frame_id
    traj.joint_names = [f"joint{i}" for i in range(1, 7)]

    for idx, waypoint in enumerate(path):
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in waypoint]
        point.time_from_start = rospy.Duration.from_sec(max(dt * idx, 0.0))
        traj.points.append(point)

    return traj


class SequenceController:
    def __init__(self, open_paths_file: Path, close_paths_file: Path, path_index: int, dt: float):
        self.open_paths = load_paths(open_paths_file)
        self.close_paths = load_paths(close_paths_file)
        self.path_index = path_index
        self.dt = dt
        self.done_event = Event()
        self.last_done = None

        self.open_pub = rospy.Publisher("/ur5/open_path", JointTrajectory, queue_size=1, latch=True)
        self.close_pub = rospy.Publisher("/ur5/close_path", JointTrajectory, queue_size=1, latch=True)
        self.gripper_pub = rospy.Publisher("/ur5/gripper_cmd", Bool, queue_size=1, latch=True)
        self.done_sub = rospy.Subscriber("/ur5/execution_done", String, self._done_cb)

    def _done_cb(self, msg: String):
        self.last_done = msg.data
        self.done_event.set()

    def _wait_for_done(self, timeout: float = 120.0):
        self.done_event.clear()
        if not self.done_event.wait(timeout=timeout):
            raise RuntimeError("Timed out waiting for /ur5/execution_done")

    def _publish_gripper(self, close: bool):
        self.gripper_pub.publish(Bool(data=close))

    def _publish_path(self, pub, path, label: str):
        if not path:
            raise RuntimeError(f"{label} path is empty")
        pub.publish(path_to_trajectory(path, self.dt))
        rospy.loginfo("Published %s path with %d waypoints", label, len(path))

    def run(self):
        if self.path_index < 0:
            indices = range(min(len(self.open_paths), len(self.close_paths)))
        else:
            indices = [self.path_index]

        rospy.loginfo("Waiting for executor to subscribe and start...")
        rospy.sleep(0.5)

        for idx in indices:
            if idx >= len(self.open_paths) or idx >= len(self.close_paths):
                raise IndexError(
                    f"path_index {idx} out of range for open/close path files "
                    f"({len(self.open_paths)} open, {len(self.close_paths)} close)"
                )

            rospy.loginfo("Executing sequence %d", idx)
            self._publish_gripper(close=False)
            self._publish_path(self.open_pub, self.open_paths[idx], f"open[{idx}]")
            self._wait_for_done()

            self._publish_gripper(close=True)
            rospy.sleep(max(self.dt, 0.1))
            self._publish_path(self.close_pub, self.close_paths[idx], f"close[{idx}]")
            self._wait_for_done()

            self._publish_gripper(close=False)
            rospy.sleep(max(self.dt, 0.1))

        rospy.loginfo("All requested path indices completed.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--open-paths-file",
        default=str(DEFAULT_PROJECT_DIR / "open_paths.json"),
    )
    parser.add_argument(
        "--close-paths-file",
        default=str(DEFAULT_PROJECT_DIR / "close_paths.json"),
    )
    parser.add_argument("--path-index", type=int, default=0)
    parser.add_argument("--dt", type=float, default=0.05)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ur5_sequence_controller", anonymous=False)
    open_paths_file = rospy.get_param("~open_paths_file", args.open_paths_file)
    close_paths_file = rospy.get_param("~close_paths_file", args.close_paths_file)
    path_index = int(rospy.get_param("~path_index", args.path_index))
    dt = float(rospy.get_param("~dt", args.dt))

    rospy.loginfo("Loading open paths from %s", open_paths_file)
    rospy.loginfo("Loading close paths from %s", close_paths_file)

    controller = SequenceController(
        Path(open_paths_file),
        Path(close_paths_file),
        path_index,
        dt,
    )
    controller.run()


if __name__ == "__main__":
    main()
