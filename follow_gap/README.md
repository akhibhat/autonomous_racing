# Reactive method: Follow the Gap

We implement the follow the gap algorithm such that the car follows the mid point of the largest gap in its path.

## Design Choices
The following choices were made to get optimal performance:
- A bubble radius of 0.4m was set
- Laser scans were clipped to a maximum limit of 5m
- A gap threshold of 2m was used, i.e. to find the longest gap, we find the maximum non-zero sequence with values above 2m.
- The best point to follow is the mid-point of the start and end indices of the longest gap.
- The steering angle is linearly interpolated with respect to the distance of the closest point.

## Usage

Launch the node along with the simulator by using the command `roslaunch akhilesh_reactive reactive_gap_follow.py`.

## Dependencies
- sensor_msgs
- ackermann_msgs
- f110_simulator

## Video

The video can be found at the [link](https://youtu.be/MbgIoJIT6lI)
