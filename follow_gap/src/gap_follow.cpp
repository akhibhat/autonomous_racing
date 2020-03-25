#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class FollowGap
{
    private:
        ros::NodeHandle nh_;

        // Publisher
        ros::Publisher drive_pub_;

        // Subscriber
        ros::Subscriber scan_sub_;

        // Other variables
        double car_length_;
        double bubble_radius_;
        double scan_threshold_;
        double gap_threshold_;

    public:
        
        FollowGap(ros::NodeHandle &nh)
        {
            nh_ = nh;
            car_length_ = 0.50;
            bubble_radius_ = 0.15 + car_length_/2;
            scan_threshold_ = 3.0;
            gap_threshold_ = 1.5;
        }

        ~FollowGap(){}

        void initialize()
        {
            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);

            scan_sub_ = nh_.subscribe("/scan", 1, &FollowGap::laserScanCallback, this);
        }

        std::vector<double> filterScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            std::vector<double> proc_scan;
            
            for (int i=0; i<scan_msg->ranges.size(); i++)
            {
                if (std::isnan(scan_msg->ranges[i]))
                {
                    proc_scan.push_back(0.0);
                }
                else if (scan_msg->ranges[i] > scan_threshold_)
                {
                    proc_scan.push_back(scan_threshold_);
                }
                else
                {
                    proc_scan.push_back(scan_msg->ranges[i]);
                }
            }

            return proc_scan;
        }

        size_t find_best_point(const size_t start_idx, const size_t end_idx, std::vector<double>& proc_ranges)
        {
            const auto best_point = (start_idx + end_idx)/2;

            return best_point;
        }

        void eliminate_bubble(std::vector<double>* input_vector, const double increment, double closest_pt, size_t closest_idx)
        {
            auto angle = bubble_radius_/closest_pt;

            auto start_idx = round(closest_idx - (angle/increment));
            auto end_idx = round(closest_idx + (angle/increment));

            if (end_idx > input_vector->size())
            {
                end_idx = input_vector->size() - 1;
            }

            if (start_idx < 0)
            {
                start_idx = 0;
            }

            for (int i=start_idx; i <= end_idx; i++)
            {
                input_vector->at(i) = 0.0;
            }
        }

        std::pair<size_t, size_t> find_max_gap(const std::vector<double>& input_vector)
        {
            size_t max_start_idx = 0;
            size_t current_idx = 0;
            size_t max_size_ = 0;
            size_t current_start;
            size_t current_size;

            while (current_idx < input_vector.size())
            {
                current_start = current_idx;
                current_size = 0;

                while (current_idx < input_vector.size() && input_vector[current_idx] > gap_threshold_)
                {
                    current_size++;
                    current_idx++;
                }

                if (current_size > max_size_)
                {
                    max_start_idx = current_start;
                    max_size_ = current_size;
                    current_size = 0;
                }
                current_idx++;
            }

            if (current_size > max_size_)
            {
                max_start_idx = current_start;
                max_size_ = current_size;
            }

            return {max_start_idx, max_start_idx + max_size_ - 1};
        }

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            const auto increment = scan_msg->angle_increment;

            auto proc_ranges = filterScan(scan_msg);
            ROS_DEBUG("Processed lidar scans");

            const auto closest_pt_it = std::min_element(proc_ranges.begin(), proc_ranges.end());
            auto closest_idx = std::distance(proc_ranges.begin(), closest_pt_it);

            auto closest_pt = scan_msg->ranges[closest_idx];

            eliminate_bubble(&proc_ranges, increment, closest_pt, closest_idx);

            const auto [start_idx, end_idx] = find_max_gap(proc_ranges);

            std::cout << "Start index: " << start_idx << "\n";
            std::cout << "End index: " << end_idx << "\n";
//
//            const auto best_idx = find_best_point(start_idx, end_idx, proc_ranges);
//
//            ackermann_msgs::AckermannDriveStamped drive_msg;
//            drive_msg.header = scan_msg->header;
//
//            // Calculate the best index, given best index
//            if (best_idx < scan_msg->ranges.size()/2)
//            {
//                drive_msg.drive.steering_angle = (best_idx - scan_msg->ranges.size()/2) * increment;
//            }
//            else
//            {
//                drive_msg.drive.steering_angle = (best_idx - scan_msg->ranges.size()/2) * increment;
//            }
//
//            // Use linear interpolation to avoid oscillations during race
//            if (closest_pt <= 0.5)
//            {
//                drive_msg.drive.steering_angle *= 1.0;
//            }
//            else if (closest_pt > 0.5 && closest_pt <= 1.0)
//            {
//                drive_msg.drive.steering_angle *= 0.75;
//            }
//            else if (closest_pt < 1.0 && closest_pt <=2.0)
//            {
//                drive_msg.drive.steering_angle *= 0.50;
//            }
//            else if (closest_pt > 2.0)
//            {
//                drive_msg.drive.steering_angle *= 0.25;
//            }
//
//            // Constraints on speed when compared to steering angle
//            if (abs(drive_msg.drive.steering_angle) <= 0.1745)
//            {
//                drive_msg.drive.speed = 1.50;
//            }
//            else if (abs(drive_msg.drive.steering_angle) > 0.1745 && abs(drive_msg.drive.steering_angle) < 0.3491)
//            {
//                drive_msg.drive.speed = 1.00;
//            }
//            else
//            {
//                drive_msg.drive.speed = 0.50;
//            }
//
//            // Constraints on steering angle to avoid servo locking
//            if (drive_msg.drive.steering_angle > 0.40)
//            {
//                drive_msg.drive.steering_angle = 0.40;
//            }
//            else if (drive_msg.drive.steering_angle < -0.40)
//            {
//                drive_msg.drive.steering_angle = -0.40;
//            }

            // drive_pub_.publish(drive_msg);
        }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "gap_follow");

    ros::NodeHandle nh;

    FollowGap fg(nh);
    fg.initialize();

    ros::spin();

    return 0;
}
