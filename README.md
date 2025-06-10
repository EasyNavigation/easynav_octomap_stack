# easynav_octomap_stack

This stack is part of the Easy Navigation (EasyNav) project developed by the Intelligent Robotics Lab. It provides a ROS 2 lifecycle node for building and publishing Octomap-based 3D occupancy maps from sensor point cloud data.

It is composed of the **octomap_maps_builder** package and the **octomap_maps_manager** package (currently under construction).

The `OctomapMapsBuilderNode` subscribes to sensor point cloud topics, processes and filters the data, and incrementally builds an Octomap representation of the environment. It publishes the resulting maps as ROS messages in both binary and full map formats.


## Installation

Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easynav_octomap_maps_builder
```

## Usage

Source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```
Run the lifecycle node:
```bash
ros2 run easynav_octomap_maps_builder octomap_maps_builder_node
```

## Parameters

| Parameter               | Type   | Default      | Description                                    |
|-------------------------|--------|--------------|------------------------------------------------|
| `sensor_topic`          | string | `"map"`      | Topic name for incoming sensor point clouds.  |
| `downsample_resolution` | double | `1.0`        | Downsampling resolution for input point clouds.|
| `perception_default_frame` | string | `"map"`  | Default target frame for perception fusion.   |
| `sensor_model.max_range`| double | `90.0`       | Maximum sensor range for raycasting.           |
| `resolution`            | double | `1.0`        | Resolution of the OctoMap octree.               |
| `base_frame_id`         | string | `"base_link"`| Robot base frame ID.                            |
| `occupancy_min_z`       | double | `0.1`        | Minimum z height for occupancy filtering.      |
| `occupancy_max_z`       | double | `10.0`       | Maximum z height for occupancy filtering.      |
| `publish_binary_map`    | bool   | `true`       | Enable publishing of binary octomap message.   |
| `publish_full_map`      | bool   | `true`       | Enable publishing of full octomap message.     |
| `world_frame_id`        | string | `"map"`      | Global coordinate frame for map integration.   |
