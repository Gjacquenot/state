cmake_minimum_required(VERSION 3.16)
project(vessel_info)

find_package(gz-sim REQUIRED)
find_package(GTest REQUIRED)

enable_testing()

add_executable(vessel_test vessel.cpp vessel_test.cpp)
target_link_libraries(vessel_test PRIVATE gz-sim::gz-sim GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(vessel_test)