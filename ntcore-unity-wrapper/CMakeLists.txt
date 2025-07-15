cmake_minimum_required(VERSION 3.31)
project(ntcore_unity_wrapper)

set(CMAKE_CXX_STANDARD 20)

find_package(wpilib REQUIRED)

add_library(ntcore_unity_wrapper SHARED library.cpp)
target_link_libraries(ntcore_unity_wrapper ntcore)
