# easynav_octomap_stack

This stack is part of the Easy Navigation (EasyNav) project developed by the Intelligent Robotics Lab. Ir building elevation grid maps
from point cloud data. The node subscribes to a sensor topic, processes the incoming point cloud data, downsamples it, and
publishes a grid_map_msgs::msg::GridMap message. 

It is composed of the **octomap_maps_builder** package and the **octomap_maps_manager** package (currently under construction).

The `OctomapMapsBuilderNode` subscribes to sensor point cloud topics, processes and filters the data, and incrementally builds an Octomap representation of the environment. It publishes the resulting maps as ROS messages in both binary and full map formats.


## Installation

Clone the repository into your ROS 2 workspace. Temporarilly you will need to install octomap-ros from a ThirdParties:

```bash
cd ~/ros2_ws/src
git clone https://github.com/EasyNavigation/easynav_octomap_stack.git
vcs import < easynav_octomap_stack/thirdparty.repos
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easynav_octomap_maps_builder
```

## Usage

Source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

Create a parameter YAML file (e.g., `params.yaml`) with the following content:

```yaml
octomap_maps_builder_node:
  ros__parameters:
    use_sim_time: true
    sensors: [map]
    downsample_resolution: 0.1
    perception_default_frame: map
    map:
      topic: map
      type: sensor_msgs/msg/PointCloud2
      group: points
```

Run the node using the parameter file with this command:
```
ros2 run easynav_octomap_maps_builder octomap_maps_builder_main \
--ros-args --params-file src/easynav_octomap_stack/params.yaml
```

## Parameters

| Parameter               | Type   | Default      | Description                                    |
|-------------------------|--------|--------------|------------------------------------------------|
| `sensors`          | list | -      | Topic names for incoming sensor point clouds.  |
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

