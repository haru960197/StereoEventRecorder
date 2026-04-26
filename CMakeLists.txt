cmake_minimum_required(VERSION 3.5)
project(stereo_event_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(MetavisionHAL REQUIRED)
find_package(Threads REQUIRED)

add_executable(stereo_event_recorder main.cpp)

target_link_libraries(stereo_event_recorder PRIVATE Metavision::HAL_discovery Threads::Threads)
