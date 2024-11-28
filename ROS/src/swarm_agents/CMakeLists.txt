cmake_minimum_required(VERSION 3.8)
project(swarm_agents)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(GTest REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(CURL REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(swarm src/swarm_agent_node.cpp
                src/swarm_ros.cpp
                src/http_client.cpp
                

                )

target_link_libraries(swarm
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY} # serialization
  ${OpenCV_LIBRARIES}
  pcl_common 
  pcl_io
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
  gtest
  CURL::libcurl
)

target_include_directories(swarm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(swarm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(swarm 
        rclcpp  
        sensor_msgs    
        PCL 
        std_msgs 
        geometry_msgs 
        nav_msgs 
        tf2 
        tf2_ros
        tf2_eigen
        tf2_geometry_msgs
        tf2_msgs
        cv_bridge          
)

install(TARGETS swarm
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch
  models
 
  DESTINATION share/${PROJECT_NAME}/
)

  install(DIRECTORY
  config
  
  DESTINATION share/${PROJECT_NAME}
  #FILES_MATCHING PATTERN "*.yaml" 
)

install(DIRECTORY
  config/rviz
  
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.rviz" 
)

ament_package()
