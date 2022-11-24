cmake_minimum_required(VERSION 3.10)

project(cfr_fsm)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(cfr_fsm main.cpp src/cfr_fsm.cpp)
target_link_libraries(cfr_fsm PRIVATE Threads::Threads)