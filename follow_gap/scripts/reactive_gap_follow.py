#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped as ADS

class FollowGap:
    def __init__(self):

        self.car_length = 0.5
        self.lidar_range = 360
        self.bubble_radius = (self.car_length/2) + 0.15

        self.scan_threshold = 5
        self.gap_threshold = 2

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher("/nav", ADS, queue_size=1)
#        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_1", ADS, queue_size=1)

    def truncate_scan(self, ranges):

        zero_idx = ((self.lidar_range/2) - 90) * len(ranges)/self.lidar_range
        pi_idx = ((self.lidar_range)/2 + 90) * len(ranges)/self.lidar_range

        return ranges[zero_idx:pi_idx+1]

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = []

        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                proc_ranges.append(0)
            elif ranges[i] > self.scan_threshold:
                proc_ranges.append(self.scan_threshold)
            else:
                proc_ranges.append(ranges[i])

        return proc_ranges

    def eliminate_bubble(self, proc_ranges, dist, index):
        """ Return the array of ranges after eliminating the points inside the bubble radius
        """
        angle = self.bubble_radius/dist
        start_idx = round(index - (angle/self.increment))
        end_idx = round(index + (angle/self.increment))

        if end_idx >= len(proc_ranges):
            end_idx = len(proc_ranges)-1

        if start_idx < 0:
            start_idx = 0

        for i in range(int(start_idx), int(end_idx)+1):
            proc_ranges[i] = 0

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_start_idx = 0
        max_size = 0

        current_idx = 0
        current_start = 0

        while current_idx < len(free_space_ranges):

            current_size = 0
            current_start = current_idx

            while current_idx < len(free_space_ranges) and free_space_ranges[current_idx] > self.gap_threshold:
                current_size += 1
                current_idx += 1

            if current_size > max_size:
                max_start_idx = current_start
                max_size = current_size
                current_size = 0

            current_idx += 1

        if current_size > max_size:
            max_start_idx = current_start
            max_size = current_size

        return max_start_idx, max_start_idx+max_size-1

    def find_best_point(self, start_idx, end_idx, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        chunk = ranges[start_idx:end_idx+1]

        # Returns the mid point of the gap
        idx = (start_idx + end_idx)/2
        # max_gap = max(chunk)
        # idx = chunk.index(max_gap) + start_idx + 1

        return idx

    def lidar_callback(self, scan_msg):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = scan_msg.ranges
        self.increment = scan_msg.angle_increment

        #Truncate scans to get only 180 degree scan values
        truncate_ranges = self.truncate_scan(ranges)

        proc_ranges = self.preprocess_lidar(truncate_ranges)

        #Find closest point to LiDAR
        closest_point = min(proc_ranges)
        closest_index = proc_ranges.index(closest_point)

        #Eliminate all points inside 'bubble' (set them to zero)
        post_bubble = self.eliminate_bubble(proc_ranges, closest_point, closest_index)

        #Find max length gap
        start_idx, end_idx = self.find_max_gap(post_bubble)

        #Find the best point in the gap
        final_idx = self.find_best_point(start_idx, end_idx, post_bubble)

        #Publish Drive message
        drive_msg = ADS()
        drive_msg.header = scan_msg.header

        #Calculate steering angle, given best index
        if final_idx < len(post_bubble)/2:
            goal_steering_angle = (final_idx - len(post_bubble)/2) * self.increment
        else:
            goal_steering_angle = (final_idx - len(post_bubble)/2) * self.increment

        #Use linear interpolation to avoid oscillations
        if closest_point <= 0.5:
            drive_msg.drive.steering_angle = goal_steering_angle
        elif closest_point > 0.5 and closest_point <= 1.0:
            drive_msg.drive.steering_angle = goal_steering_angle * 0.85
        elif closest_point > 1.0 and closest_point <= 2.0:
            drive_msg.drive.steering_angle = goal_steering_angle * 0.50
        elif closest_point > 2.0:
            drive_msg.drive.steering_angle = goal_steering_angle * 0.25

        #Constraints on speed when compared to steering angle
        if abs(drive_msg.drive.steering_angle) < 0.1745:
            drive_msg.drive.speed = 4
        elif abs(drive_msg.drive.steering_angle) > 0.1745 and abs(drive_msg.drive.steering_angle) < 0.3491:
            drive_msg.drive.speed = 1.5
        else:
            drive_msg.drive.speed = 1.0

        #Constraints on steering angle to avoid servo locking
        if drive_msg.drive.steering_angle > 0.43:
            drive_msg.drive.steering_angle = 0.43
        elif drive_msg.drive.steering_angle < -0.43:
            drive_msg.drive.steering_angle = -0.43

        self.drive_pub.publish(drive_msg)

if __name__ == "__main__":
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = FollowGap()
    rospy.Rate(60)
    while not rospy.is_shutdown():
        rospy.spin()
