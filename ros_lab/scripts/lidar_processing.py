#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ros_lab.msg import ScanRange

class LidarProcessing:
    def __init__(self):

        rospy.init_node("lidar_processor_node", anonymous=True)

        # Initialize the publishers
        self.cp_pub = rospy.Publisher("/closest_point", Float64, queue_size=10)
        self.fp_pub = rospy.Publisher("/farthest_point", Float64, queue_size=10)
        self.scan_pub = rospy.Publisher("/scan_range", ScanRange, queue_size=10)

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

        scan_msg = ScanRange()
        scan_msg.header = msg.header
        scan_msg.closest_point = closest_point
        scan_msg.farthest_point = farthest_point

        self.scan_pub.publish(scan_msg)

if __name__ == "__main__":
    LP = LidarProcessing()
    rospy.spin()
