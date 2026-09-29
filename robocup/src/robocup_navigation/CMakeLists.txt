cmake_minimum_required(VERSION 3.0.2)
project(robocup_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS geometry_msgs gazebo_msgs mavros_msgs nav_msgs rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/plan_training_route.py
  scripts/uav_navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
