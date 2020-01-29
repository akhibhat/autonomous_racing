#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped as ADS

class Safety():
    def __init__(self):

        rospy.init_node("safety_node", anonymous=True)
        self.brake_pub = rospy.Publisher("/brake", ADS, queue_size=1)
        self.brake_bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=1)

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.brake_bool = False

        self.brake_msg = ADS()
        self.speed = 0
        self.t_min = 0.25

    def odom_callback(self, odom_msg):

        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        ttc_min = float('inf')
        scan_ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        increment = scan_msg.angle_increment

        current_angle = angle_min

        for i in range(len(scan_ranges)):

            if not math.isnan(scan_ranges[i]) and not math.isinf(scan_ranges[i]) and self.speed:
                r_dot = -self.speed*math.cos(current_angle)

                max_r_dot = max(-r_dot, 0)

                current_ttc = scan_ranges[i]/max_r_dot if max_r_dot else float('inf')

#                if current_angle < 0.025 and current_angle > -0.025:
#                    print(current_ttc)
#
                if current_ttc < ttc_min:
                    ttc_min = current_ttc
#
            current_angle = current_angle + increment
#
        print(ttc_min)

#        if ttc_min < self.t_min:
#            print("Inside loop")
#            #self.brake_msg.drive.steering_angle = 0
#            #self.brake_msg.drive.steering_angle_velocity = 0
#            self.brake_msg.header = scan_msg.header
#            self.brake_msg.drive.speed = 0
#
#            self.brake_bool = True
#            self.brake_bool_pub.publish(self.brake_bool)
#            self.brake_pub.publish(self.brake_msg)
#        else:
#            self.brake_bool = False
#            self.brake_bool_pub.publish(self.brake_bool)


if __name__ == "__main__":
    safe = Safety()

    rospy.Rate(100)

    rospy.spin()
