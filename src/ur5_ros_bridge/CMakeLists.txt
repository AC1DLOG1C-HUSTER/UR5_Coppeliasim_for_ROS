cmake_minimum_required(VERSION 3.0.2)
project(ur5_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ur5_sequence_controller.py
  scripts/ur5_coppelia_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
