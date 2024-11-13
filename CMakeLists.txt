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

# Add at the beginning of your CMakeLists.txt, after project()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-fexec-charset=UTF-8)
endif()

# For additional UTF-8 support, you might also want to add:
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -fexec-charset=UTF-8
        -finput-charset=UTF-8
    )
endif()

add_library(http_client http_client.cpp)

# Add source files
# add_executable(test tests/base_tests.cpp)
add_executable(swarm main.cpp)  # Add the main program
add_executable(agent_handoff agent_handoff.cpp)

# Link against Google Test and project libraries
# target_link_libraries(test GTest::gtest GTest::gtest_main)

# Link CURL and other dependencies for the main program
find_package(CURL REQUIRED)
target_link_libraries(swarm PRIVATE http_client CURL::libcurl )
target_link_libraries(agent_handoff PRIVATE http_client CURL::libcurl)

# Include directories
# target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(swarm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(agent_handoff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
