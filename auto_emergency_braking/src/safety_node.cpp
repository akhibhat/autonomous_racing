#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

class Safety
{
    public:
        Safety(ros::NodeHandle &nh): rate_(120)    
        {
            nh_ = nh;
            relative_speed_ = 0.0;
            ttc_threshold_ = 0.20;
        }

        ~Safety(){}
        
        void initialize(){

            brake_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
            brake_bool_pub_ = nh_.advertise<std_msgs::Bool>("/brake_bool", 1);

            odom_sub_ = nh_.subscribe("/odom", 1, &Safety::odometryCallback, this);
            scan_sub_ = nh_.subscribe("/scan", 1, &Safety::laserScanCallback, this);
        }

        // Functions
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
            relative_speed_ = odom_msg->twist.twist.linear.x;
        }

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
            
            auto current_angle = scan_msg->angle_min;
            const auto increment = scan_msg->angle_increment;
            double min_ttc = std::numeric_limits<double>::max();

            for (int i=0; i < scan_msg->ranges.size(); i++)
            {
                if(!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i]))
                {
                    const double current_ttc = scan_msg->ranges[i] / std::max(0.0, relative_speed_ * cos(current_angle));

//                    std::cout << current_ttc << "\n";
                    if (current_ttc < min_ttc)
                    {
                        min_ttc = current_ttc;
                    }
                }

                current_angle += increment;
            }

            if (min_ttc < ttc_threshold_)
            {
                ackermann_msgs::AckermannDriveStamped brake_msg;
                brake_msg.header = scan_msg->header;
                brake_msg.drive.speed = 0.0;

                std_msgs::Bool brake_bool_msg;
                brake_bool_msg.data = true;

                brake_bool_pub_.publish(brake_bool_msg);
                brake_pub_.publish(brake_msg);
            }
            else
            {
                std_msgs::Bool brake_bool_msg;
                brake_bool_msg.data = false;
                brake_bool_pub_.publish(brake_bool_msg);
            }

            rate_.sleep();
        }

    private:
        ros::NodeHandle nh_;

        // Publishers
        ros::Publisher brake_pub_;
        ros::Publisher brake_bool_pub_;

        // Subscribers
        ros::Subscriber odom_sub_;
        ros::Subscriber scan_sub_;

        // Callback rate
        ros::Rate rate_;

        // Other variables
        double relative_speed_;
        double ttc_threshold_;

};

int main(int argc, char **argv){

    ros::init(argc, argv, "AEB_node");

    ros::NodeHandle nh;

    Safety sf(nh);
    sf.initialize();

    ros::spin();

    return 0;
}
