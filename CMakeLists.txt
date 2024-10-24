cmake_minimum_required(VERSION 3.10)
project(swarm.cpp)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and include nlohmann_json
include_directories(/home/liyang/Dependencies/nlohmann-json/include/)

# Exclude Anaconda paths
set(CMAKE_IGNORE_PATH "/opt/anaconda3")
# Find Google Test
find_package(GTest REQUIRED CONFIG PATHS /usr/local/lib/cmake/GTest NO_DEFAULT_PATH)
# message(STATUS "GTest include dirs: ${GTEST_INCLUDE_DIRS}")
# message(STATUS "GTest libraries: ${GTEST_LIBRARIES}")

# Add source files
add_executable(util_test tests/util_test.cpp)

# Link against Google Test and project libraries
target_link_libraries(util_test GTest::gtest GTest::gtest_main)

# Include directories
target_include_directories(util_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
