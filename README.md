# heading_controller_pkg

ROS 2 (Humble) C++ package with four nodes: heading/PID steering controller,
GPS distance & velocity node, and two GPS speed (accelerator) controllers.

## Build

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select heading_controller_pkg
source install/setup.bash
```

## Test

```bash
colcon test --packages-select heading_controller_pkg
colcon test-result --all --verbose
```

## Run

```bash
ros2 run heading_controller_pkg heading_controller --ros-args -p waypoint_csv:=/data/clean.csv
ros2 run heading_controller_pkg gps_distance_velocity_node
ros2 run heading_controller_pkg gps_speed_controller
ros2 run heading_controller_pkg gps_speed_controller_alert
```

## Requirements

ROS 2 Humble, `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`

```bash
rosdep install --from-paths src --ignore-src -r -y
```
