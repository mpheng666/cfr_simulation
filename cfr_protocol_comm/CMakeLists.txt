cmake_minimum_required(VERSION 3.5)
project(cfr_protocol_comm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cfr_state_machine REQUIRED)
find_package(cfr_protocol_interfaces)

include_directories( include )

# ===== protocol_server_node =====
add_executable(protocol_server_node node/protocol_server_node.cpp src/protocol_server.cpp)
target_include_directories(protocol_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(protocol_server_node rclcpp cfr_protocol_interfaces cfr_state_machine)
target_link_libraries(protocol_server_node)

install(TARGETS protocol_server_node
  DESTINATION lib/${PROJECT_NAME})

# ===== protocol_client_node =====
add_executable(protocol_client_node node/protocol_client_node.cpp src/protocol_client.cpp)
target_include_directories(protocol_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(protocol_client_node rclcpp cfr_protocol_interfaces)
target_link_libraries(protocol_client_node)

install(TARGETS protocol_client_node
  DESTINATION lib/${PROJECT_NAME})


# ===== protocol_client library =====
# add_library(protocol_client src/protocol_client.cpp)
# target_include_directories(protocol_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(protocol_client rclcpp cfr_protocol_interfaces)

# ament_export_targets(protocol_client HAS_LIBRARY_TARGET)
# ament_export_dependencies(rclcpp std_msgs std_srvs cfr_protocol_interfaces)
# install(
#   DIRECTORY include/cfr_protocol_comm
#   DESTINATION include
# )
# install(
#   TARGETS protocol_client
#   EXPORT protocol_client
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
