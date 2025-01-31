cmake_minimum_required(VERSION 3.8)
project(planner)

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


# Alternatively, set global include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
  clipper
)

set(CLIPPER_SRC
  clipper/clipper.cpp
)



add_executable(main_combinatorial
  src/main_combinatorial.cpp
  src/combinatorial_based/map_construction.cpp
  src/combinatorial_based/marker_publishers/map_edges_publisher.cpp
  src/combinatorial_based/marker_publishers/PointMarker.cpp

  src/combinatorial_based/map_utilities.cpp
    src/combinatorial_based/path_utilities.cpp

  src/combinatorial_based/utilities.cpp
  src/combinatorial_based/shortest_graph.cpp

  src/map_generator.cpp
  src/utils/dubins.cpp

  ${CLIPPER_SRC}
)

  ament_target_dependencies(main_combinatorial
  rclcpp
  geometry_msgs
  visualization_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav2_msgs
)

install(TARGETS
  main_combinatorial
  DESTINATION lib/${PROJECT_NAME}
)

# ament_target_dependencies(planner rclcpp geometry_msgs obstacles_msgs)

# install(TARGETS
#   planner
#   DESTINATION lib/${PROJECT_NAME}
# )

###################
#### GRAPH_GEN ####
###################

add_executable(main_sampling 
  src/main_sampling.cpp
  src/map_generator.cpp
  src/sampling_based/utils/rrt.cpp
  src/sampling_based/utils/kdtree.cpp
  src/utils/dubins.cpp
  src/utils/orchestrator.cpp
  )

ament_target_dependencies(main_sampling
  rclcpp
  geometry_msgs
  visualization_msgs
  obstacles_msgs
  tf2
  tf2_ros
  nav2_msgs
  )


  
  target_link_libraries(main_sampling Boost::headers)
  target_include_directories(main_sampling PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  
  
  install(TARGETS
  main_sampling
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

install(DIRECTORY 
        launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
