# Automatic Emergency Braking (AEB)

We implement a safety node for the car that will stop the car from collision while travelling at high velocities. Time to Collision (TTC) calculated from the LaserScan message is used to implement this feature.

## Working
The safety node listens to the `\odom` and `\scan` topic to calculate the TTC using the formula,


![Alt text](https://github.com/akhibhat/autonomous_racing/raw/master/auto_emergency_braking/ttc.gif)



Here, _r_ is the distance between the two objects which is obtained from the laser scan message. In the denominator,we take the max of 0 and the velocity in the direction of the scan.

## Parameters
The `ttc_threshold_` parameter in `safety_node.cpp` needs to be tuned if operating on the actual car. The current value is set for the simulator.

## Dependencies
- sensor_msgs
- nav_msgs
- ackermann_msgs
- std_msgs
