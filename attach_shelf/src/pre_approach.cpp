#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "unistd.h"


using namespace std::chrono_literals;
using scan = sensor_msgs::msg::LaserScan;
using robot_vel = geometry_msgs::msg::Twist;
using robot_odom = nav_msgs::msg::Odometry;
using namespace std::placeholders;

class PreApproach : public rclcpp::Node
{
    public:
        PreApproach(int& argc, char** argv) : Node("pre_approach_node")
        {
            rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
            
            if (true) {
                //access parameter from launch file
                this->declare_parameter("obstacles", 0.0);
                this->declare_parameter("degrees", 0);
                
                get_params();
            
            } else {
            
                 //////////////Argument conversion//////////////////////////
                std::string distance_to_obstacle_arg = argv[2];
                std::string rotation_degree_arg = argv[4];
                distance_to_obstacle = std::stof(distance_to_obstacle_arg);
                rotation_degree = std::stoi(rotation_degree_arg);
            
            
            }
            
            //////////////Group callback//////////////////////////
            scanner_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions option1;
            option1.callback_group = scanner_group_;

            timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            
            //////////////Implement publisher & subscriber//////////////////////////
            vel_pub_ = this->create_publisher<robot_vel>("robot/cmd_vel", 10);
            robot_vel vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub_->publish(vel_msg);

            scanner_sub_ = this->create_subscription<scan>("scan", 1, std::bind(&PreApproach::laser_scan_callback, this, _1), option1);
            odom_sub_ = this->create_subscription<robot_odom>("odom", 10, std::bind(&PreApproach::odom_callback, this, _1), option1);
            timer_ = this->create_wall_timer(50ms, std::bind(&PreApproach::move_robot, this), timer_group_);

            //RCLCPP_DEBUG(this->get_logger(), "distance to obstacle: %s", distance_to_obstacle_arg.c_str());
            //RCLCPP_DEBUG(this->get_logger(), "rotation degree: %s", rotation_degree_arg.c_str());
        }

        void get_params()
        {
        
            distance_to_obstacle =
                this->get_parameter("obstacles").get_parameter_value().get<float>();
            rotation_degree =
                this->get_parameter("degrees").get_parameter_value().get<int>();

            RCLCPP_INFO(this->get_logger(), "GET PARAMS, DISTANCE %f", distance_to_obstacle);
            RCLCPP_INFO(this->get_logger(), "GET PARAMS, DEGREES %d", rotation_degree);
            
        

        }
        
    private:

        void laser_scan_callback(const scan::SharedPtr msg)
        {
            //check range size
            int range_size = msg->ranges.size();
            //RCLCPP_DEBUG(this->get_logger(), "Range Size Scanner: %d", range_size);
            //get front range
            front_range = msg->ranges[range_size / 2];
            RCLCPP_DEBUG(this->get_logger(), "Front Scanner Range: %f", front_range);
        
        }

        void odom_callback(const robot_odom::SharedPtr msg)
        {
            tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            tf2::Matrix3x3 mat(quat);

            double r; double p;
            mat.getRPY(r, p, yaw);
            RCLCPP_DEBUG(this->get_logger(), "Yaw(rad): %f", yaw);
        }

        void move_robot()
        {
            RCLCPP_DEBUG(this->get_logger(), "Moving robot. Distance to wall:= %f", front_range);
            robot_vel vel_msg;
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0.0;
            vel_pub_->publish(vel_msg);

            if (front_range <= distance_to_obstacle)
            {
                timer_->cancel();
                timer_ = this->create_wall_timer(50ms, std::bind(&PreApproach::rotate_robot, this), timer_group_);
            
            }

        }

        void rotate_robot()
        {
            RCLCPP_DEBUG(this->get_logger(), "Rotating robot");

            float deg_to_rad = rotation_degree * M_PI / 180.0;
            RCLCPP_WARN(this->get_logger(), "target rotation: %f, current rotation: %f", deg_to_rad, yaw);

            robot_vel vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = -0.2;
            vel_pub_->publish(vel_msg);

            if (float(yaw) <= deg_to_rad)
            {
                
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                vel_pub_->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(), "Target Achieved");
                timer_->cancel();
                
            }
            
        }


        float distance_to_obstacle;
        int rotation_degree;
        double yaw;
        rclcpp::CallbackGroup::SharedPtr scanner_group_{nullptr};
        rclcpp::Subscription<scan>::SharedPtr scanner_sub_{nullptr};
        rclcpp::Publisher<robot_vel>::SharedPtr vel_pub_{nullptr};
        rclcpp::Subscription<robot_odom>::SharedPtr odom_sub_{nullptr};
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        rclcpp::CallbackGroup::SharedPtr timer_group_{nullptr};
        float front_range;
        



};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto pre_approach_node = std::make_shared<PreApproach>(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pre_approach_node);
    executor.spin();


    rclcpp::shutdown();
    return 0;
}