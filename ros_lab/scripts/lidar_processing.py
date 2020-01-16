#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class LidarProcessing:
    def __init__(self):

        rospy.init_node("lidar_processor_node", anonymous=True)

        # Initialize the publishers
        self.cp_pub = rospy.Publisher("/closest_point", Float64, queue_size=10)
        self.fp_pub = rospy.Publisher("/farthest_point", Float64, queue_size=10)

        # Initialize the subscriber
        rospy.Subscriber("scan", LaserScan, self.lidar_cb)

    def lidar_cb(self, msg):

        ranges = msg.ranges
        closest_point = min(ranges)
        farthest_point = max(ranges)

        cp = Float64()
        cp.data = closest_point

        fp = Float64()
        fp.data = farthest_point

        self.cp_pub.publish(cp)
        self.fp_pub.publish(fp)

if __name__ == "__main__":
    LP = LidarProcessing()
    rospy.spin()
