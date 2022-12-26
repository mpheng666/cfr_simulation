cmake_minimum_required(VERSION 3.5)
project(cfr_navigation)

find_package(catkin REQUIRED roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
