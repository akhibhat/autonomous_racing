#include <ros_lab/lidar_processing.h>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "lidar_processing");

    ros::NodeHandle nh;

    LidarProcessing lp(nh);
    lp.initialize();

    ros::spin();

    return 0;
}
