#add_executable(test tests/base_tests.cpp)
add_executable(base_agent agents/base_agent.cpp)
add_executable(dynamic_agent agents/dynamic_agent.cpp)
add_executable(weather_agent agents/weather_agent.cpp)
add_executable(agent_handoff agents/agent_handoff.cpp)
add_executable(function_calling openai/function_calling.cpp)
add_executable(openai_cpp_usage openai/openai_cpp_usage.cpp)
add_executable(use_database database/use_database.cpp)


# Link CURL and other dependencies for the main program
target_link_libraries(base_agent PRIVATE http_client CURL::libcurl)
target_link_libraries(dynamic_agent PRIVATE http_client CURL::libcurl)
target_link_libraries(weather_agent PRIVATE http_client CURL::libcurl )
target_link_libraries(agent_handoff PRIVATE http_client CURL::libcurl)
target_link_libraries(function_calling PRIVATE http_client CURL::libcurl)
target_link_libraries(openai_cpp_usage PRIVATE http_client CURL::libcurl)
target_link_libraries(use_database PRIVATE http_client CURL::libcurl)

# Link against Google Test and project libraries
# target_link_libraries(test GTest::gtest GTest::gtest_main)


# Include directories
# target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(base_agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(dynamic_agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(weather_agent PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(agent_handoff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(function_calling PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(openai_cpp_usage PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}) 
target_include_directories(use_database PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
