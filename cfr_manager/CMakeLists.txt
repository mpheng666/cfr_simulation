cmake_minimum_required(VERSION 3.5)
project(cfr_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ===== cfr_manager =====
add_library(cfr_manager src/cfr_manager.cpp)
target_include_directories(cfr_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfr_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs)

ament_export_targets(cfr_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs)
install(
  DIRECTORY include/cfr_manager
  DESTINATION include
)
install(
  TARGETS cfr_manager
  EXPORT cfr_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
