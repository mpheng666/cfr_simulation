cmake_minimum_required(VERSION 3.5)
project(cfr_feedback_server)

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
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

include_directories( include ${Boost_INCLUDE_DIRS} )

# ===== cfr_feedback_server lib =====
add_library(cfr_feedback_server src/cfr_feedback_server.cpp)
target_include_directories(cfr_feedback_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfr_feedback_server Boost)

ament_export_targets(cfr_feedback_server HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
install(
  DIRECTORY include/cfr_feedback_server
  DESTINATION include
)
install(
  TARGETS cfr_feedback_server
  EXPORT cfr_feedback_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ===== cfr_feedback_client =====
add_executable(cfr_feedback_client_node node/cfr_feedback_client_node.cpp)
target_include_directories(cfr_feedback_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cfr_feedback_client_node
)
target_link_libraries(cfr_feedback_client_node ${Boost_LIBRARIES} PRIVATE Threads::Threads)

install(TARGETS cfr_feedback_client_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
