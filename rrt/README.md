# RRT* - A sampling-based motion planning algorithm

This package implements both RRT and RRT* algorithms for motion planning. You can choose to enable or disable RRT* by setting values of `enable_rrt_star_` variable in the file `rrt.cpp`

## Usage

Make sure to have the F1/10 simulator before you run this package
- Launch the simulator with `roslaunch team4_rrt_star rrt.launch`
- Launch the rrt node with `rosrun team4_rrt_star team4_rrt_star_node`

## Dependencies
- sensor_msgs
- geometry_msgs
- visualization_msgs
- std_msgs
- ackermann_msgs
- tf
- tf2_ros
- tf2_geometry_msgs
- f110_simulator

## Video
- The video can be found on the link [link](https://youtu.be/utUAKjqXxJI)
