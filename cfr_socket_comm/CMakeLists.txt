cmake_minimum_required(VERSION 3.5)
project(cfr_socket_comm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
# find_package(cfr_protocol_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories( include ${Boost_INCLUDE_DIRS} )

# ===== cfr_mock_command_server_node =====
add_executable(cfr_mock_command_server_node node/cfr_mock_command_server_node.cpp src/cfr_socket_server.cpp)
target_include_directories(cfr_mock_command_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_mock_command_server_node
  rclcpp
)

# ===== cfr_mock_feedback_server_node =====
add_executable(cfr_mock_feedback_server_node node/cfr_mock_feedback_server_node.cpp src/cfr_mock_feedback_server.cpp)
target_include_directories(cfr_mock_feedback_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_mock_feedback_server_node
  rclcpp
)

# ===== cfr_manual_command_client_node =====
add_executable(cfr_manual_command_client_node node/cfr_manual_command_client_node.cpp src/cfr_socket_client.cpp)
target_include_directories(cfr_manual_command_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_manual_command_client_node
  rclcpp
)

# ===== cfr_feedback_client_node =====
add_executable(cfr_feedback_client_node node/cfr_feedback_client_node.cpp src/cfr_feedback_client.cpp)
target_include_directories(cfr_feedback_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_feedback_client_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# ===== cfr_auto_command_client_node =====
add_executable(cfr_auto_command_client_node node/cfr_auto_command_client_node.cpp src/cfr_auto_command_client.cpp)
target_include_directories(cfr_auto_command_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_auto_command_client_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS cfr_mock_command_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cfr_mock_feedback_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cfr_manual_command_client_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cfr_feedback_client_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cfr_auto_command_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
