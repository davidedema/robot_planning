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
find_package(obstacles_msgs REQUIRED)

# add_executable(graph_generator src/graph_node.cpp)
# ament_target_dependencies(graph_generator rclcpp geometry_msgs obstacles_msgs)

# install(TARGETS
#   graph_generator
#   DESTINATION lib/${PROJECT_NAME}
# )

###############
#### TESTS ####
###############

add_executable(test_graph_generator 
  src/simple_node.cpp
  src/graph_node.cpp
  )

ament_target_dependencies(test_graph_generator
  rclcpp
  geometry_msgs
  obstacles_msgs
)
target_include_directories(test_graph_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  test_graph_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
