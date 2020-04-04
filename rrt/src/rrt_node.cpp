#include "rrt/rrt.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    
    RRT rrt(nh);
    ros::Rate r(60);
    ros::spin();

    return 0;
}
