# ROSLAB

A basic introduction to ROS with implementation of subscribers, publishers (both python and c++), message files, launch files and rviz.

## Files

- `lidar_processing.cpp` is the C++ file that listens to the `/scan` topic and publishes a custom message consisting of a header along with the closest and farthest point from the laser scans
- `lidar_processing.py` is the Python file does the same
- `scan_range.msg` is the custom message consisting of
    - `Header`
    - 2 floats for the closest and farthest point
- `roslab.rviz` is the config file for rviz that helps visualize the `/map` and the `/scan` topic
- `roslab.launch` file that launches rviz and the two lidar processing nodes.

## Dependencies
- `sensor_msgs`
- `std_msgs`

