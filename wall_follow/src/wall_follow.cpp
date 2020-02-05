#include <ros/ros.h>
#include <wall_follow/safety_node.cpp>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class WallFollow
{
    private:
        ros::NodeHandle nh_;

        // Publisher
        ros::Publisher drive_pub_;

        // Subscriber
        ros::Subscriber scan_sub_;

        double kp_, kd_, ki_;

        double right_d_, left_d_;
        double integral_;
        
        double prev_time_, current_time_;
        
        int angle_range_, theta_;
        double theta_rad_;

        double velocity_;
        double alpha_;
        double lookahead_;
        double error_, prev_error_;

        bool follow_left_, follow_right_;

    public:
        WallFollow(ros::NodeHandle &nh)
        {
            nh_ = nh;
            
            // PID variables
            kp_ = 4.0;
            kd_ = 0.0;
            ki_ = 0.0;
    
            // Wall follow params
            right_d_ = 0.9;
            left_d_ = 0.55;
            velocity_ = 2.0;
            angle_range_ = 360;
            theta_ = 70;
            theta_rad_ = 1.2217;
            alpha_ = 0.0;
            lookahead_ = 2.0;
    
            // Other variables
            prev_error_ = 0.0;
            error_ = 0.0;
            integral_ = 0.0;
            follow_left_ = true;
            follow_right_ = false;
            prev_time_ = 0.0;
            current_time_ = 0.0;
        }

        ~WallFollow(){}

        void initialize(){

            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

            scan_sub_ = nh_.subscribe("/scan", 1, &WallFollow::laserScanCallback, this);
        }

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

            // make_decision(scan_msg->ranges);
            if (follow_left_ == true)
            {
                followLeft(scan_msg);
            }
            else
            {
                followRight(scan_msg);
            }

            pid_control();
        }

        void followLeft(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

            int zero_index, theta_index;
            double b, a, dt;
            
            zero_index = ((angle_range_/2) + 90) * (scan_msg->ranges.size())/angle_range_;
            theta_index = ((angle_range_/2) + 90 - theta_) * (scan_msg->ranges.size())/angle_range_;

            b = scan_msg->ranges[zero_index];
            a = scan_msg->ranges[theta_index];

            alpha_ = std::atan2(a * cos(theta_rad_) - b, a * sin(theta_rad_));

            dt = b * cos(alpha_) + lookahead_ * sin(alpha_);

            error_= -(left_d_ - dt);
            
            std::cout << dt << "\n";
        }
        
        void followRight(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

            int zero_index, theta_index;
            double b, a, dt;
            
            zero_index = ((angle_range_/2) - 90) * (scan_msg->ranges.size())/angle_range_;
            theta_index = ((angle_range_/2) - 90 + theta_) * (scan_msg->ranges.size())/angle_range_;

            b = scan_msg->ranges[zero_index];
            a = scan_msg->ranges[theta_index];

            alpha_ = std::atan2(a * cos(theta_rad_) - b, a * sin(theta_rad_));

            dt = b * cos(alpha_) + lookahead_ * sin(alpha_);

            error_= right_d_ - dt;
        }

        void pid_control(){
            
            prev_time_ = current_time_;
            current_time_ = ros::Time::now().toSec();

            double time_diff = current_time_ - prev_time_;

            integral_ += error_;
            double steering_angle = kp_*error_; //+ ki_*integral_ + kd_*((error_ - prev_error_)/time_diff);

            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "laser";

            if(std::isnan(steering_angle)){
                drive_msg.drive.speed = 0;
                drive_msg.drive.steering_angle = 0;
                ROS_ERROR("Invalid steering angle! Cannot be nan");
            }

            drive_msg.drive.steering_angle = steering_angle;

            if (abs(steering_angle) < 0.1745){
                drive_msg.drive.speed = 1.5;
            }
            else if(abs(steering_angle) > 0.1745 && abs(steering_angle) < 0.3491){
                drive_msg.drive.speed = 1.0;
            }
            else{
                drive_msg.drive.speed = 0.5;
            }

            drive_pub_.publish(drive_msg);

            prev_error_ = error_;
        }
};

int main(int argc, char **argv){
    
    ros::init(argc, argv, "wallFollow");

    ros::NodeHandle nh;

    WallFollow wf(nh);
    wf.initialize();

    ros::spin();

    return 0;
}
