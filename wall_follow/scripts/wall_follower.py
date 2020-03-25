#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped as  ADS

class WallFollow:

    def __init__(self):

        rospy.init_node("wall_follow_node", anonymous=True)

        self.kp = 5.0
        self.kd = 0.01
        self.ki = 0.0
        self.servo_offset = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.prev_time = 0.0
        self.current_time = 0.0
        self.integral = 0.0

        self.angle_range = 360
        self.theta = 60
        self.theta_rad = 1.0472
        self.right_d = 0.9
        self.left_d = 0.55
        self.velocity = 2.0
        self.car_length = 0.50

        self.lookahead = 0.20

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.laserScanCallback)
        self.drive_pub = rospy.Publisher("/nav", ADS, queue_size = 1)

    def laserScanCallback(self, scan_msg):

        # scan_truncated = self.truncate_scan(scan_msg)
#        alpha = self.rightWallFollow(scan_msg)
        alpha = self.leftWallFollow(scan_msg)

        if  abs(alpha) < 0.1745:
            self.velocity = 0.5
            print(self.velocity)
        elif abs(alpha) > 0.1745 and abs(alpha) < 0.3491:
            self.velocity = 0.5
            print(self.velocity)
        else:
            self.velocity = 0.5
            print(self.velocity)

        self.pid_control()

    def pid_control(self):

        self.prev_time = self.current_time
        self.current_time = rospy.Time.now().to_sec()

        drive_msg = ADS()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"

        self.integral = self.integral + self.error

        steering_angle = self.kp*self.error + self.kd*((self.error - self.prev_error)/(self.current_time-self.prev_time)) + self.integral*self.ki

        if math.isnan(steering_angle):
            print("Steering angle not valid")
        else:
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = self.velocity

        self.drive_pub.publish(drive_msg)

        self.prev_error = self.error

    def rightWallFollow(self, scan_msg):

        zero_index = ((self.angle_range/2) - 90) * (len(scan_msg.ranges)/self.angle_range)
        theta_index = ((self.angle_range/2) - 90 + self.theta) * (len(scan_msg.ranges)/self.angle_range)

        b = scan_msg.ranges[zero_index]
        a = scan_msg.ranges[theta_index]

        alpha = math.atan2(a*math.cos(self.theta_rad) - b, a*math.sin(self.theta_rad))

        dt = b*math.cos(alpha) + self.lookahead*math.sin(alpha)

        self.error = dt - self.right_d

        return alpha

    def leftWallFollow(self, scan_msg):

        zero_index = ((self.angle_range/2) + 90) * (len(scan_msg.ranges)/self.angle_range)
        theta_index = ((self.angle_range/2) + 90 - self.theta) * (len(scan_msg.ranges)/self.angle_range)

        b = scan_msg.ranges[zero_index]
        a = scan_msg.ranges[theta_index]

        alpha = math.atan2(a*math.cos(self.theta_rad) - b, a*math.sin(self.theta_rad))

        dt = b*math.cos(alpha) + self.lookahead*math.sin(alpha)

        self.error = -(self.left_d - dt)

        return alpha

if __name__ == "__main__":
    wf = WallFollow()

    while not rospy.is_shutdown():
        rospy.spin()
