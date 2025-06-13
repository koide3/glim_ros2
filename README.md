# glim_ros2

Please open issues in the main GLIM repository: https://github.com/koide3/glim

[![ROS2](https://github.com/koide3/glim_ros2/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ros2/actions/workflows/build.yml)

## Parameters

This section details the ROS parameters available for configuring the `glim_ros` node.

### Initial Pose Configuration

When loading a pre-existing map, you can provide an initial pose for the robot to start localization. This helps GLIM orient itself within the loaded map. The initial pose can be specified either via launch file parameters or by publishing to the `/initialpose` topic (e.g., using the "2D Pose Estimate" tool in RViz).

**Launch File Parameters:**

To set the initial pose via a launch file, you can add the following parameters to your `glim_ros` node configuration:

```xml
<node pkg="glim_ros" exec="glim_rosnode" name="glim_ros">
    <!-- Other parameters -->
    <param name="initial_pose.position.x" value="1.2"/>
    <param name="initial_pose.position.y" value="3.4"/>
    <param name="initial_pose.position.z" value="0.5"/>
    <param name="initial_pose.orientation.x" value="0.0"/>
    <param name="initial_pose.orientation.y" value="0.0"/>
    <param name="initial_pose.orientation.z" value="0.707"/>
    <param name="initial_pose.orientation.w" value="0.707"/>
    <!-- Other parameters -->
</node>
```

*   `initial_pose.position.[x|y|z]`: The initial position of the robot in meters (default: 0.0).
*   `initial_pose.orientation.[x|y|z|w]`: The initial orientation of the robot as a quaternion (default: x=0, y=0, z=0, w=1.0).

**RViz:**

Alternatively, you can set the initial pose dynamically using RViz by publishing a `geometry_msgs/PoseWithCovarianceStamped` message to the `/initialpose` topic. The "2D Pose Estimate" tool in RViz can be used for this purpose (ensure the topic it publishes to is remapped or set to `/initialpose`).

### Localization Launch File (`glim_localize.launch.py`)

For convenience, especially when starting localization with a pre-built map and a known initial pose, a dedicated launch file `glim_localize.launch.py` is provided. This launch file allows you to specify the map path and initial pose directly as command-line arguments.

**Usage:**

```bash
ros2 launch glim_ros glim_localize.launch.py \
    map_path:=/path/to/your/glim_map_directory \
    initial_pose_x:=1.5 \
    initial_pose_y:=-0.5 \
    initial_pose_z:=0.0 \
    initial_pose_qx:=0.0 \
    initial_pose_qy:=0.0 \
    initial_pose_qz:=0.0 \
    initial_pose_qw:=1.0
```

**Launch Arguments:**

*   `map_path` (string): **Required.** Full path to the directory containing the GLIM map data you want to load.
*   `initial_pose_x` (double, default: "0.0"): Initial robot X position in meters.
*   `initial_pose_y` (double, default: "0.0"): Initial robot Y position in meters.
*   `initial_pose_z` (double, default: "0.0"): Initial robot Z position in meters.
*   `initial_pose_qx` (double, default: "0.0"): X component of the initial robot orientation quaternion.
*   `initial_pose_qy` (double, default: "0.0"): Y component of the initial robot orientation quaternion.
*   `initial_pose_qz` (double, default: "0.0"): Z component of the initial robot orientation quaternion.
*   `initial_pose_qw` (double, default: "1.0"): W component of the initial robot orientation quaternion.

This launch file will configure the `glim_ros` node to load the specified map and apply the initial pose upon startup. You can still use RViz to send a new initial pose via the `/initialpose` topic if needed.

Note: IMU and LiDAR topic names should be configured using the standard GLIM configuration files (e.g., within the `config` directory, typically in `config_ros.json`).
