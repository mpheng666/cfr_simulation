cmake_minimum_required(VERSION 3.5)
project(cfr_socket_comm)

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
find_package(Boost REQUIRED)
find_package(cfr_manager REQUIRED)
find_package(cfr_state_machine REQUIRED)

include_directories( include ${Boost_INCLUDE_DIRS} )

add_executable(cfr_socket_server_node node/cfr_socket_server_node.cpp src/cfr_socket_server.cpp)
target_include_directories(cfr_socket_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_socket_server_node
  rclcpp
  std_msgs
  std_srvs
  cfr_state_machine
)

add_executable(cfr_socket_client_node node/cfr_socket_client_node.cpp src/cfr_socket_client.cpp)
target_include_directories(cfr_socket_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_socket_client_node
  rclcpp
  std_msgs
)

install(TARGETS cfr_socket_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cfr_socket_client_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
