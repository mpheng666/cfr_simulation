cmake_minimum_required(VERSION 3.5)
project(cfr_navigation)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
