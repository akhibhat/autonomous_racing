# Automatic Emergency Braking (AEB)

We implement a safety node for the car that will stop the car from collision while travelling at high velocities. Time to Collision (TTC) calculated from the LaserScan message is used to implement this feature.

## Working
The safety node listens to the `\odom` and `\scan` topic to calculate the TTC using the formula,



Here, _r_ is the distance between the two objects and 

## Parameters
The `ttc_threshold_` parameter in `safety_node.cpp` needs to be tuned if operating on the actual car. The current value is set for the simulator.

## Dependencies
- sensor_msgs
- nav_msgs
- ackermann_msgs
- std_msgs
