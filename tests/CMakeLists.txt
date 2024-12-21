cmake_minimum_required(VERSION 3.8)
project(tests)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost 1.74)
find_package(CGAL-6.0.1)


# Include directories
include_directories(include)

# Add library
add_library(my_marker_library
    src/goal_start_marker_publisher.cpp
)

ament_target_dependencies(my_marker_library
    rclcpp
    geometry_msgs
    visualization_msgs
)

# Export the library
ament_export_include_directories(include)
ament_export_libraries(my_marker_library)

# Add the executable
add_executable(main
    src/tests.cpp
)

# Link the library to the executable
target_link_libraries(main my_marker_library)

# Specify compile features for the executable
target_compile_features(main PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
