// Copyright 2025 Intelligent Robotics Lab
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the OctomapMapsBuilderNode class.

#ifndef EASYNAV_OCTOMAP_MAPS_BUILDER__OCTOMAPMAPSBUILDERNODE_HPP_
#define EASYNAV_OCTOMAP_MAPS_BUILDER__OCTOMAPMAPSBUILDERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "easynav_common/types/Perceptions.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

namespace easynav
{

  /**
   * @class OctomapMapsBuilderNode
   * @brief ROS2 lifecycle node for building and maintaining 3D occupancy maps using OctoMap.
   *
   * This node subscribes to sensor data, processes perception point clouds,
   * updates an internal OctoMap occupancy map, and publishes the map as binary
   * or full OctoMap messages. It supports lifecycle management callbacks for
   * configuration, activation, deactivation, and cleanup.
   */
class OctomapMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(OctomapMapsBuilderNode)   ///< Smart pointer definitions for this node.

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief Constructor.
     * @param options Node initialization options.
     */
  explicit OctomapMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destructor.
     */
  ~OctomapMapsBuilderNode();

    /**
     * @brief Lifecycle callback invoked when node is configured.
     *
     * Initializes parameters, declares topics, and sets up data structures.
     * @param state The current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Lifecycle callback invoked when node is activated.
     *
     * Activates publishers and starts processing data.
     * @param state The current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Lifecycle callback invoked when node is deactivated.
     *
     * Stops publishing data and pauses processing.
     * @param state The current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Lifecycle callback invoked when node is cleaned up.
     *
     * Releases resources and resets internal state.
     * @param state The current lifecycle state.
     * @return CallbackReturnT indicating success or failure.
     */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

    /**
     * @brief Perform a processing cycle to update the map.
     *
     * Processes incoming perception data, updates the OctoMap, and publishes results.
     * Should be called periodically.
     */
  void cycle();

    /**
     * @brief Registers a perception handler.
     * @param handler Shared pointer to a PerceptionHandler instance.
     */
  void register_handler(std::shared_ptr<PerceptionHandler> handler);

protected:
    /**
     * @brief Update the minimum bounding box key based on an input key.
     *
     * Compares each coordinate and stores the minimum in `min`.
     * @param in Input OcTreeKey.
     * @param min Minimum key to update.
     */
  inline static void updateMinKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & min)
  {
    for (size_t i = 0; i < 3; ++i) {
      min[i] = std::min(in[i], min[i]);
    }
  }

    /**
     * @brief Update the maximum bounding box key based on an input key.
     *
     * Compares each coordinate and stores the maximum in `max`.
     * @param in Input OcTreeKey.
     * @param max Maximum key to update.
     */
  inline static void updateMaxKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & max)
  {
    for (size_t i = 0; i < 3; ++i) {
      max[i] = std::max(in[i], max[i]);
    }
  }

private:
  using PCLPoint = pcl::PointXYZ;                    ///< PCL point type alias.
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;   ///< PCL point cloud alias.
  using OcTreeT = octomap::OcTree;                   ///< Octomap OcTree type alias.

    /**
     * @brief Callback to insert new sensor point cloud data.
     *
     * Processes incoming point clouds, filters and transforms them,
     * then inserts into the octree map.
     */
  void insert_cloud_callback();

    /**
     * @brief Insert a processed nonground point cloud scan into the octree.
     * @param nonground Point cloud excluding ground points.
     */
  void insert_scan(const PCLPointCloud & nonground);

    /**
     * @brief Publish octomap messages.
     *
     * Publishes either binary or full octomap messages according to node parameters.
     */
  void publish();

    /// Frame ID of the robot's base.
  std::string base_frame_id_;

    /// Frame ID of the world coordinate system.
  std::string world_frame_id_;

    /// Resolution for downsampling the input cloud.
  double downsample_resolution_;

    /// Maximum range to consider points for the map.
  double max_range_;

    /// Resolution of the octomap voxels.
  double resolution_;

    /// Minimum height (Z) for occupancy consideration.
  double occupancy_min_z_;

    /// Maximum height (Z) for occupancy consideration.
  double occupancy_max_z_;

    /// Whether to publish the binary octomap.
  bool publish_binary_map_;

    /// Whether to publish the full octomap.
  bool publish_full_map_;

    /// Unique pointer to the OctoMap octree instance.
  std::unique_ptr<OcTreeT> octree_;

    /// Key ray used for raycasting in octomap updates.
  octomap::KeyRay key_ray_;

    /// Current octree depth.
  unsigned tree_depth_;

    /// Maximum allowed depth for the octree.
  unsigned max_tree_depth_;

    /// Minimum key for bounding box update in the octree.
  octomap::OcTreeKey update_bbox_min_;

    /// Maximum key for bounding box update in the octree.
  octomap::OcTreeKey update_bbox_max_;

    /// Map of perception data grouped by sensor name.
  std::map<std::string, std::vector<PerceptionPtr>> perceptions_;

    /// Registered perception handlers by sensor name.
  std::map<std::string, std::shared_ptr<PerceptionHandler>> handlers_;

    /// Minimum X coordinate filter for incoming clouds.
  double point_cloud_min_x_{-100.0};

    /// Maximum X coordinate filter for incoming clouds.
  double point_cloud_max_x_{100.0};

    /// Minimum Y coordinate filter for incoming clouds.
  double point_cloud_min_y_{-100.0};

    /// Maximum Y coordinate filter for incoming clouds.
  double point_cloud_max_y_{100.0};

    /// Minimum Z coordinate filter for incoming clouds.
  double point_cloud_min_z_{-10.0};

    /// Maximum Z coordinate filter for incoming clouds.
  double point_cloud_max_z_{10.0};

    /// Lifecycle publisher for binary octomap messages.
  rclcpp_lifecycle::LifecyclePublisher<octomap_msgs::msg::Octomap>::SharedPtr pub_binary_;

    /// Lifecycle publisher for full octomap messages.
  rclcpp_lifecycle::LifecyclePublisher<octomap_msgs::msg::Octomap>::SharedPtr pub_full_;

    /// Callback group to handle subscription callbacks.
  rclcpp::CallbackGroup::SharedPtr cbg_;

    /// Cached point cloud of ground points.
  PCLPointCloud pc_ground;

    /// Cached point cloud of nonground points.
  PCLPointCloud pc_nonground;

    /// Sensor model probability of hit.
  double prob_hit_{0.7};

    /// Sensor model probability of miss.
  double prob_miss_{0.4};

    /// Minimum clamping threshold for occupancy probability.
  double clamping_thres_min_{0.12};

    /// Maximum clamping threshold for occupancy probability.
  double clamping_thres_max_{0.97};
};

} // namespace easynav

#endif // EASYNAV_OCTOMAP_MAPS_BUILDER__OCTOMAPMAPSBUILDERNODE_HPP_
