#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

class LidarProcessing
{
    public:
        LidarProcessing(ros::NodeHandle &nh);
        ~LidarProcessing();
        void initialize();

    private:
        ros::NodeHandle nh_;

        // Publishers
        ros::Publisher closest_point_pub_;
        ros::Publisher farthest_point_pub_;
        ros::Publisher scan_msg_pub_;

        //Subscriber
        ros::Subscriber scan_sub_;

        // Subscriber Callback
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};
