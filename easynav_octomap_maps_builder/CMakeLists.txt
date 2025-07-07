cmake_minimum_required(VERSION 3.20)
project(easynav_octomap_maps_builder)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(easynav_common REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)

# Main library
add_library(${PROJECT_NAME} SHARED
  src/easynav_octomap_maps_builder/OctomapMapsBuilderNode.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${octomap_ros_INCLUDE_DIRS}
)

# Link against ROS2 targets and external libs
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${octomap_msgs_TARGETS}
  ${octomap_ros_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${OCTOMAP_LIBRARIES}
  ${OCTOMATH_LIBRARIES}
  easynav_common::easynav_common
  pcl_conversions::pcl_conversions
  pcl_ros::bag_to_pcd_lib
  pcl_ros::combined_pointcloud_to_pcd_lib
  pcl_ros::pcd_to_pointcloud_lib
  pcl_ros::pcl_ros_filter
  pcl_ros::pcl_ros_filters
  pcl_ros::pcl_ros_tf
  pcl_ros::pointcloud_to_pcd_lib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# Install library and headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Main executable
add_executable(octomap_maps_builder_main src/octomap_maps_builder_main.cpp)
target_link_libraries(octomap_maps_builder_main PUBLIC
  ${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

# Export for downstream
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  easynav_common
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  visualization_msgs
)

ament_package()
