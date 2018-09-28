cmake_minimum_required(VERSION 2.8.3)
project(assistant_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS assistant_gazebo turtlebot3_slam
)

install(FILES
  launch/rviz.launch
  launch/simulation.launch
  launch/view_urdf.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
