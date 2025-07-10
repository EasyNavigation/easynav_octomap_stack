ament_add_gtest(octomap_maps_builder_tests octomap_maps_builder_tests.cpp)

target_include_directories(octomap_maps_builder_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(octomap_maps_builder_tests
  ${PROJECT_NAME}
  ${octomap_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
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
