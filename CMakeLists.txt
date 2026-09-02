cmake_minimum_required(VERSION 3.16)
project(vessel_info)

find_package(gz-sim REQUIRED)   # transitively brings in gz-math headers/libs

add_executable(vessel_info main.cpp)
target_link_libraries(vessel_info gz-sim::gz-sim)
