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
/// \brief Implementation of the OctomapMapsBuilderNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_octomap_maps_builder/OctomapMapsBuilderNode.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.hpp>
#include <octomap/OcTreeKey.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace easynav
{

OctomapMapsBuilderNode::OctomapMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("octomap_maps_builder_node", options)
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("downsample_resolution")) {
    declare_parameter("downsample_resolution", 1.0);
  }

  if (!has_parameter("sensor_model.max_range")) {
    declare_parameter("sensor_model.max_range", 90.0);
  }

  if (!has_parameter("resolution")) {
    declare_parameter("resolution", 1.0);
  }

  if (!has_parameter("base_frame_id")) {
    declare_parameter("base_frame_id", "base_link");
  }

  if (!has_parameter("occupancy_min_z")) {
    declare_parameter("occupancy_min_z", 0.1);
  }

  if (!has_parameter("occupancy_max_z")) {
    declare_parameter("occupancy_max_z", 10.0);
  }

  if (!has_parameter("publish_binary_map")) {
    declare_parameter("publish_binary_map", true);
  }

  if (!has_parameter("publish_full_map")) {
    declare_parameter("publish_full_map", true);
  }

  if (!has_parameter("world_frame_id")) {
    declare_parameter("world_frame_id", "map");
  }

  pub_binary_ = create_publisher<octomap_msgs::msg::Octomap>(
        "map_builder/octomap_binary", rclcpp::QoS(1).transient_local().reliable());

  pub_full_ = create_publisher<octomap_msgs::msg::Octomap>(
        "map_builder/octomap_full", rclcpp::QoS(1).transient_local().reliable());

  register_handler(std::make_shared<PointPerceptionHandler>());
}

OctomapMapsBuilderNode::~OctomapMapsBuilderNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
OctomapMapsBuilderNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("downsample_resolution", downsample_resolution_);
  get_parameter("sensor_model.max_range", max_range_);
  get_parameter("resolution", resolution_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("occupancy_min_z", occupancy_min_z_);
  get_parameter("occupancy_max_z", occupancy_max_z_);
  get_parameter("publish_binary_map", publish_binary_map_);
  get_parameter("publish_full_map", publish_full_map_);

  get_parameter("world_frame_id", world_frame_id_);

    // Initialize OctoMap
  octree_ = std::make_unique<OcTreeT>(resolution_);
  octree_->setProbHit(prob_hit_);
  octree_->setProbMiss(prob_miss_);
  octree_->setClampingThresMin(clamping_thres_min_);
  octree_->setClampingThresMax(clamping_thres_max_);

  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, group;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }
    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", group);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);
    get_parameter(sensor_id + ".group", group);

    RCLCPP_DEBUG(get_logger(),
                   "Loaded sensor parameters: id=%s topic=%s type=%s group=%s",
                   sensor_id.c_str(), topic.c_str(), msg_type.c_str(), group.c_str());

    auto handler_it = handlers_.find(group);
    if (handler_it == handlers_.end()) {
      RCLCPP_WARN(get_logger(), "No handler for group [%s]", group.c_str());
      continue;
    }

    auto ptr = handler_it->second->create(sensor_id);
    auto sub = handler_it->second->create_subscription(*this, topic, msg_type, ptr, cbg_);

    perceptions_[group].emplace_back(PerceptionPtr{ptr, sub});

    RCLCPP_DEBUG(get_logger(), "Creating perception for sensor %s", sensor_id.c_str());
    RCLCPP_DEBUG(get_logger(), "Handler group = %s", group.c_str());
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OctomapMapsBuilderNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  if (publish_binary_map_) {
    pub_binary_->on_activate();
  }
  if (publish_full_map_) {
    pub_full_->on_activate();
  }
  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
OctomapMapsBuilderNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  if (publish_binary_map_) {
    pub_binary_->on_deactivate();
  }
  if (publish_full_map_) {
    pub_full_->on_deactivate();
  }
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OctomapMapsBuilderNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  octree_.reset();

  if (publish_binary_map_) {
    pub_binary_.reset();
  }
  if (publish_full_map_) {
    pub_full_.reset();
  }

  return CallbackReturnT::SUCCESS;
}

void OctomapMapsBuilderNode::insert_cloud_callback()
{
  auto point_perceptions = get_point_perceptions(perceptions_["points"]);
  auto processed_perceptions = PointPerceptionsOpsView(point_perceptions);
    // Fuse perceptions if the frame_id is different from default and downsample
  if (!point_perceptions.empty() && point_perceptions[0] &&
    point_perceptions[0]->frame_id != world_frame_id_)
  {
    processed_perceptions.downsample(downsample_resolution_).fuse(world_frame_id_);
  } else {
    processed_perceptions.downsample(downsample_resolution_);
  }

  PCLPointCloud pc = processed_perceptions.as_points();

    // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);

    // just filter height range:
  pass_x.setInputCloud(pc.makeShared());
  pass_x.filter(pc);
  pass_y.setInputCloud(pc.makeShared());
  pass_y.filter(pc);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

  pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
  pc_ground.header = pc.header;
  pc_nonground.header = pc.header;

  insert_scan(pc_nonground);
}

void OctomapMapsBuilderNode::insert_scan(const PCLPointCloud & nonground)
{
  tf2::Vector3 position_tf(0, 0, 0);

  const auto sensor_origin = octomap::pointTfToOctomap(position_tf);

  if (!octree_->coordToKeyChecked(sensor_origin, update_bbox_min_) ||
    !octree_->coordToKeyChecked(sensor_origin, update_bbox_max_))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for origin " << sensor_origin);
  }

  octomap::KeySet free_cells, occupied_cells;

  for (const auto & point_pcl : nonground) {
    octomap::point3d point(point_pcl.x, point_pcl.y, point_pcl.z);

    if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_)) {
      if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
      }

      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key)) {
        occupied_cells.insert(key);
        updateMinKey(key, update_bbox_min_);
        updateMaxKey(key, update_bbox_max_);
      }
    } else {
      octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
      if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());

        octomap::OcTreeKey end_key;
        if (octree_->coordToKeyChecked(new_end, end_key)) {
          free_cells.insert(end_key);
          updateMinKey(end_key, update_bbox_min_);
          updateMaxKey(end_key, update_bbox_max_);
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << new_end);
        }
      }
    }
  }

  for (const auto & free_key : free_cells) {
    if (occupied_cells.find(free_key) == occupied_cells.end()) {
      octree_->updateNode(free_key, false);
    }
  }

  for (const auto & occ_key : occupied_cells) {
    octree_->updateNode(occ_key, true);
  }

  auto min_pt = octree_->keyToCoord(update_bbox_min_);
  auto max_pt = octree_->keyToCoord(update_bbox_max_);

  RCLCPP_DEBUG_STREAM(get_logger(), "Updated bounding box: " << min_pt << " - " << max_pt);

  octree_->prune();
}

void OctomapMapsBuilderNode::publish()
{

  auto point_perceptions = get_point_perceptions(perceptions_["points"]);
  const size_t octomap_size = octree_->size();

  if (octomap_size <= 1) {
    RCLCPP_WARN(get_logger(), "Nothing to publish, octree is empty");
    return;
  }

  if ((pub_binary_->get_subscription_count() +
    pub_binary_->get_intra_process_subscription_count() >
    0))
  {
    octomap_msgs::msg::Octomap map;
    map.header.frame_id = world_frame_id_;
    map.header.stamp = point_perceptions[0]->stamp;

    if (octomap_msgs::binaryMapToMsg(*octree_, map)) {
      pub_binary_->publish(map);
    } else {
      RCLCPP_ERROR(get_logger(), "Error serializing OctoMap (binary)");
    }
  }

  if ((pub_full_->get_subscription_count() +
    pub_full_->get_intra_process_subscription_count() >
    0))
  {
    octomap_msgs::msg::Octomap map;
    map.header.frame_id = world_frame_id_;
    map.header.stamp = point_perceptions[0]->stamp;

    if (octomap_msgs::fullMapToMsg(*octree_, map)) {
      pub_full_->publish(map);
    } else {
      RCLCPP_ERROR(get_logger(), "Error serializing OctoMap (full)");
    }
  }
}

void OctomapMapsBuilderNode::cycle()
{
    // Finish cycle if no new perceptions
  if (std::none_of(perceptions_["points"].begin(), perceptions_["points"].end(),
    [](const auto & p)
    {return p.perception->new_data;}))
  {
    return;
  }

  if ((pub_binary_->get_subscription_count() +
    pub_binary_->get_intra_process_subscription_count()) > 0 ||
    (pub_full_->get_subscription_count() + pub_full_->get_intra_process_subscription_count()) > 0)
  {

    RCLCPP_INFO(get_logger(), "MAIN SUBS");
    insert_cloud_callback();
    publish();

      // Mark perceptions as not new after published
    for (auto & p : perceptions_["points"]) {
      if (p.perception->new_data) {
        p.perception->new_data = false;
      }
    }
  }
}
void
OctomapMapsBuilderNode::register_handler(std::shared_ptr<PerceptionHandler> handler)
{
  handlers_[handler->group()] = handler;
}
} // namespace easynav
