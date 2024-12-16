cmake_minimum_required(VERSION 3.8)
project(graph_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost 1.74)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# add_executable(graph_generator src/graph_node.cpp)
# ament_target_dependencies(graph_generator rclcpp geometry_msgs obstacles_msgs)

# install(TARGETS
#   graph_generator
#   DESTINATION lib/${PROJECT_NAME}
# )

###################
#### GRAPH_GEN ####
###################

add_executable(entrypoint 
  src/entrypoint.cpp
  src/graph_node.cpp
  src/sampling_based/utils/rrt.cpp
  src/sampling_based/utils/kdtree.cpp
  src/utils/dubins.cpp
  )

ament_target_dependencies(entrypoint
  rclcpp
  geometry_msgs
  visualization_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav2_msgs
  )
  
  target_link_libraries(entrypoint Boost::headers)
  target_include_directories(entrypoint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  
  install(TARGETS
  entrypoint
  DESTINATION lib/${PROJECT_NAME}
  )
  
  #####################
  #### NAV2_CLIENT ####
  #####################
  
  add_executable(nav2_client 
  src/nav2_client.cpp
  )
  
  ament_target_dependencies(nav2_client
  rclcpp
  rclcpp_components
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  visualization_msgs
  obstacles_msgs
)

install(TARGETS
  nav2_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
