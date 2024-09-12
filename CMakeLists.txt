cmake_minimum_required(VERSION 3.8)
project(pointcloud_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add the executable
add_executable(publish_pc_ply src/publish_pc_ply.cpp)
add_executable(publish_pc_hap src/publish_pc_ply_with_color.cpp)

# Link against dependencies
ament_target_dependencies(publish_pc_ply rclcpp sensor_msgs pcl_conversions)
ament_target_dependencies(publish_pc_hap rclcpp sensor_msgs)

# Link the executable with the PCL and ROS 2 libraries)
target_link_libraries(publish_pc_ply ${PCL_LIBRARIES})


# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Install the executable
install(TARGETS publish_pc_ply
        DESTINATION lib/${PROJECT_NAME}
)


# Add include directories
include_directories(include)
target_include_directories(publish_pc_hap PRIVATE include) # Adjust this path to where happly.h is located
install(TARGETS publish_pc_hap
        DESTINATION lib/${PROJECT_NAME}
)

# Add non-ROS 2 executable
add_executable(standalone_ply_reader src/standalone_ply_reader.cpp)
# Specify that happly is header-only, so no additional linking is needed
target_include_directories(standalone_ply_reader PRIVATE include) # Adjust this path to where happly.h is located
# Install the non-ROS 2 executable
install(TARGETS standalone_ply_reader
        DESTINATION bin
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
