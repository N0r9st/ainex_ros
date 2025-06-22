cmake_minimum_required(VERSION 3.0.2)
project(myproject)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/action_server.py
  scripts/action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 