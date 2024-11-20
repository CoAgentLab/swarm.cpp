cmake_minimum_required(VERSION 3.10)
project(swarm.cpp)

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

# requires at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(OPENAI_BUILD_EXAMPLES "Build example programs" ON)

# Find dependencies
find_package(CURL REQUIRED)
# find_package(nlohmann_json REQUIRED)
# include_directories(/path/to/nlohmann-json/include/)
find_package(GTest REQUIRED CONFIG PATHS /usr/local/lib/cmake/GTest NO_DEFAULT_PATH)
# message(STATUS "GTest include dirs: ${GTEST_INCLUDE_DIRS}")
# message(STATUS "GTest libraries: ${GTEST_LIBRARIES}")

# Exclude Anaconda paths
set(CMAKE_IGNORE_PATH "/opt/anaconda3")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Create library target
add_library(http_client src/http_client.cpp)


target_include_directories(http_client
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(http_client
    PUBLIC
        CURL::libcurl
        # nlohmann_json::nlohmann_json
)


# Examples
if(OPENAI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()  