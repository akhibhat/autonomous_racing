#include <vector>
#include <algorithm>
#include <iostream>
#include <typeinfo>
#include <ros_lab/lidar_processing.h>
#include <ros_lab/ScanRange.h>

LidarProcessing::LidarProcessing(ros::NodeHandle &nh){
    nh_ = nh;
}

LidarProcessing::~LidarProcessing(){}

void LidarProcessing::initialize(){
    
    closest_point_pub_ = nh_.advertise<std_msgs::Float64>("/closest_point", 1);
    farthest_point_pub_ = nh_.advertise<std_msgs::Float64>("/farthest_point", 1);
    scan_msg_pub_ = nh_.advertise<ros_lab::ScanRange>("/lidar_range", 10);
    
    scan_sub_ = nh_.subscribe("/scan", 1, &LidarProcessing::laserScanCallback, this);
}

void LidarProcessing::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    
    std::vector<float> ranges;
    std_msgs::Float64 closest_point;
    std_msgs::Float64 farthest_point;

    ros_lab::ScanRange lidar_msg;

    ranges = scan_msg->ranges;
    closest_point.data = *std::min_element(ranges.begin(), ranges.end());
    farthest_point.data = *std::max_element(ranges.begin(), ranges.end());

    closest_point_pub_.publish(closest_point);
    farthest_point_pub_.publish(farthest_point);

    lidar_msg.header = scan_msg->header;
    lidar_msg.closest_point = closest_point.data;
    lidar_msg.farthest_point = farthest_point.data;

    scan_msg_pub_.publish(lidar_msg);
}
