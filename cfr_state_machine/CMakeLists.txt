cmake_minimum_required(VERSION 3.5)
project(cfr_state_machine)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(cfr_manager REQUIRED)

include_directories( include ${Boost_INCLUDE_DIRS} )

# ===== cfr_sm_lib =====
add_library(cfr_sm_lib src/cfr_state_machine.cpp)
target_include_directories(cfr_sm_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfr_sm_lib rclcpp std_msgs std_srvs cfr_manager)

ament_export_targets(cfr_sm_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs cfr_manager)
install(
  DIRECTORY include/cfr_state_machine
  DESTINATION include
)
install(
  TARGETS cfr_sm_lib
  EXPORT cfr_sm_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ===== cfr_sm_node =====
add_executable(cfr_sm_node src/cfr_sm_node.cpp src/cfr_sm_server.cpp)
target_include_directories(cfr_sm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfr_sm_node rclcpp std_msgs std_srvs)
target_link_libraries(cfr_sm_node cfr_sm_lib)

install(TARGETS cfr_sm_node
  DESTINATION lib/${PROJECT_NAME})

# ===== cfr_sm_client =====
add_library(cfr_sm_client src/cfr_sm_client.cpp)
target_include_directories(cfr_sm_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfr_sm_client rclcpp std_msgs std_srvs)

ament_export_targets(cfr_sm_client HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs std_srvs)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS cfr_sm_client
  EXPORT cfr_sm_client
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
