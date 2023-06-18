
/*
1. It will detect the legs of the shelf using the laser intensity values (check Laser Intensities section)
2. If the laser only detects 1 shelf leg or none, it will return a False message.
3. If it detects both legs, the service will publish a transform named cart_frame to the center point between both legs.
4. Then, the robot will use this TF to move towards the shelf (using the transform coordinates).
5. Once the robot has reached the TF coordinates, it will move forward 30 cm more (to end up right underneath the shelf).
*/

//service name: /approach_shelf

//#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf_msg/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "rcutils/logging.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <unistd.h>
#include <vector>
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_msgs/msg/empty.hpp"



using go2loading = attach_shelf_msg::srv::GoToLoading;
using velocity = geometry_msgs::msg::Twist;
using laser = sensor_msgs::msg::LaserScan;
using odom = nav_msgs::msg::Odometry;
using pose = geometry_msgs::msg::Pose;
using transformStamped = geometry_msgs::msg::TransformStamped;

using namespace std::placeholders;
using namespace std::chrono_literals;

class AttachShelfServer : public rclcpp::Node
{
    public:
    AttachShelfServer() : Node("attach_server_node")
    {
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

        sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions opt;
        opt.callback_group = sub_group_;
        
        srv_ = this->create_service<go2loading>("/approach_shelf", std::bind(&AttachShelfServer::service_callback, this, _1, _2), rmw_qos_profile_services_default, srv_group_);        
        vel_pub_ = this->create_publisher<velocity>("/robot/cmd_vel", 10);
        lift_pub_ = this->create_publisher<std_msgs::msg::Empty>("elevator_up", 10);
        laser_sub_ = this->create_subscription<laser>("/scan", 1, std::bind(&AttachShelfServer::laser_callback, this, _1), opt);
        odom_sub_ = this->create_subscription<odom>("/odom",10, std::bind(&AttachShelfServer::odom_callback, this, _1), opt);
        found_shelf = false;
        
        //initialize transformation
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "START ATTACH SHELF SERVER");
    }

    private:

    void service_callback(const std::shared_ptr<go2loading::Request> req, const std::shared_ptr<go2loading::Response> res)
    {
        //request: attach_to_shelf
        //response: complete
        if (req->attach_to_shelf == true)
        {
            //publish "cart_frame" transform
            RCLCPP_INFO(this->get_logger(), "SERVICE CALLED");
                
            //initiate timer
            timer1_ = this->create_wall_timer(10ms, std::bind(&AttachShelfServer::publish_tf_shelf, this), timer_group_);
            timer2_ = this->create_wall_timer(50ms, std::bind(&AttachShelfServer::get_tf_shelf, this), timer_group_);
            usleep(1000000);

            if (found_shelf == true) {
                this->move_to_front_shelf();
                //move robot underneath the shelf
                this->move_under_shelf();
                //lift the shelf
                auto msg = std_msgs::msg::Empty();
                lift_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "FINAL APPROACH ACHIEVED");
                res->complete = true;
            } else {
                res->complete = false;
                RCLCPP_INFO(this->get_logger(), "FINAL APPROACH INCOMPLETE");
            
            }
 
        }

        if (req->attach_to_shelf == false)
        {
            //just publish the "cart_transform" and do nothing
            RCLCPP_INFO(this->get_logger(), "SERVICE CALLED");
            if (found_shelf == true) {
                timer1_ = this->create_wall_timer(10ms, std::bind(&AttachShelfServer::publish_tf_shelf, this), timer_group_);
                //RCLCPP_DEBUG(this->get_logger(), "PUBLISHED TRANSFORMATION");
                res->complete = false;
                RCLCPP_INFO(this->get_logger(), "FINAL APPROACH INCOMPLETE");
            }
            else
                res->complete = false;
                RCLCPP_INFO(this->get_logger(), "FINAL APPROACH INCOMPLETE");
            
        }
    
    
    }
    void odom_callback(const odom::SharedPtr msg)
    {
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(q);

        double r, p;
        mat.getRPY(r, p, robot_yaw);
        //RCLCPP_DEBUG(this->get_logger(),"ROBOT YAW ==> %f", robot_yaw);
    }


    void laser_callback(const laser::SharedPtr msg)
    {
        //debug laser
        int count_leg = 0;
        bool found_first_leg = false;
        bool found_second_leg = false;

        int first_leg_idx_start = 0;
        int first_leg_idx_mid = 0;
        int first_leg_idx_end = 0;
        int second_leg_idx_start = 0;
        int second_leg_idx_mid = 0;
        int second_leg_idx_end = 0;
        float first_leg_range = 0.0;
        float second_leg_range = 0.0;
        int intensity_size = msg->intensities.size();
        //RCLCPP_DEBUG(this->get_logger(), "INTENSITY LENGTH ==> %d", intensity_size);
        intensity_array = msg->intensities;
    
        //RCLCPP_DEBUG(this->get_logger(), "INTENSITY VALUES RECEIVED. Midpoint intensity: %f", intensity_array[intensity_size/2]);
        auto max_intensity = std::max_element(intensity_array.begin(),intensity_array.end());
        //RCLCPP_WARN(this->get_logger(), "MAX INTENSITY VALUE ==> %f", *max_intensity);

        //count max intensities
        for (int i=0; i < intensity_array.size(); i++)
        {

            if (count_leg == 0 && intensity_array[i-1] == 0.0 && intensity_array[i] == *max_intensity){
            
                //found 1st leg start
                first_leg_idx_start = i;
            } else if (count_leg == 0 && intensity_array[i-1] == *max_intensity && intensity_array[i] == 0.0) {
                //found 1st leg end
                first_leg_idx_end = i;
                count_leg++;
                first_leg_idx_mid = first_leg_idx_start + (first_leg_idx_end - first_leg_idx_start) / 2;
                first_leg_range = msg->ranges[first_leg_idx_mid];
            } else if (count_leg == 1 && intensity_array[i-1] == 0.0 && intensity_array[i] == *max_intensity) {
                //found 2nd leg start
                second_leg_idx_start = i;
            } else if (count_leg == 1 && intensity_array[i-1] == *max_intensity && intensity_array[i] == 0.0) {
                //found 2nd leg end
                second_leg_idx_end = i;
                count_leg++;
                second_leg_idx_mid = second_leg_idx_start + (second_leg_idx_end - second_leg_idx_start) / 2;
                second_leg_range = msg->ranges[second_leg_idx_mid];
            }

            //RCLCPP_WARN(this->get_logger(), "INTENSITY #%d ==> %f", i, intensity_array[i]);
        }

        if (count_leg <= 1) {
            found_shelf = false;
            RCLCPP_DEBUG(this->get_logger(), "FOUND SHELF: %s", found_shelf ? "yes" : "no");
        } else {
            found_shelf = true;
            RCLCPP_DEBUG(this->get_logger(), "FOUND SHELF: %s", found_shelf ? "yes" : "no");
        }

        //compute angle for 1st leg & 2nd leg in degrees
        float first_leg_angle = abs(first_leg_idx_mid - (int)msg->ranges.size()/2) * msg->angle_increment;
        float second_leg_angle = abs(second_leg_idx_mid - (int)msg->ranges.size()/2)* msg->angle_increment;
        //RCLCPP_DEBUG(this->get_logger(), "1ST LEG ANGLE: %f", first_leg_angle);
        //RCLCPP_DEBUG(this->get_logger(), "2ND LEG ANGLE: %f", second_leg_angle);
        
        //mid_dist : distance from the robot to shelf
        float mid_dist1 = cos(first_leg_angle) * first_leg_range;
        float mid_dist2 = cos(second_leg_angle) * second_leg_range;
        //RCLCPP_DEBUG(this->get_logger(), "1ST LEG MID DISTANCE: %f", mid_dist1);
        //RCLCPP_DEBUG(this->get_logger(), "2ND LEG MID DISTANCE: %f", mid_dist2);

        //distance between two legs
        float mid_leg = (sin(first_leg_angle)*first_leg_range + sin(second_leg_angle)*second_leg_range)/2.0;
        //RCLCPP_DEBUG(this->get_logger(), "MID LEG DISTANCE: %f", mid_leg);

        //translation to shelf
        tx = mid_dist1;
        ty = abs(sin(first_leg_angle) * first_leg_range - mid_leg);
        //RCLCPP_DEBUG(this->get_logger(), "TRANSLATION X,Y: %f, %f", tx, ty);

        
    }

    void publish_tf_shelf(){
    
        auto t = geometry_msgs::msg::TransformStamped();
        t.header.frame_id = "robot_front_laser_base_link";
        t.child_frame_id = "caster_frame";

        t.header.stamp = this->get_clock()->now();
        t.transform.translation.x = tx;
        t.transform.translation.y = ty;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(t);
        //RCLCPP_DEBUG(this->get_logger(), "PUBLISHED TRANSFORMATION");
    }

    void get_tf_shelf(){
    
        desired_pose = get_tf("robot_front_laser_base_link", "caster_frame");
    
    }

    pose get_tf(std::string fromFrame, std::string toFrame)
    {
        transformStamped t;
        pose robot_pose;

        try{
            t = tf_buffer_->lookupTransform(fromFrame,toFrame,tf2::TimePointZero);
        } catch (const tf2::TransformException& ex){

            //RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
            robot_pose.position.x = 0.0;
            robot_pose.position.y = 0.0;
            robot_pose.position.z = 0.0;
            robot_pose.orientation.x = 0.0;
            robot_pose.orientation.y = 0.0;
            robot_pose.orientation.z = 0.0;
            robot_pose.orientation.w = 1.0;
            //rclcpp::shutdown();
            return robot_pose;
        }
    
        auto translation_pose = t.transform.translation; 
        auto rotation_pose = t.transform.rotation;

        robot_pose.position.x = translation_pose.x;
        robot_pose.position.y = translation_pose.y;
        robot_pose.position.z = translation_pose.z;
        robot_pose.orientation.x = rotation_pose.x;
        robot_pose.orientation.y = rotation_pose.y;
        robot_pose.orientation.z = rotation_pose.z;
        robot_pose.orientation.w = rotation_pose.w;

        //std::cout << "pose x: " << robot_pose.position.x << std::endl;
        //std::cout << "pose y: " << robot_pose.position.y << std::endl;
        //std::cout << "pose z: " << robot_pose.position.z << std::endl;
        
        return robot_pose;

    }


    void move_to_front_shelf()
    {
        velocity vel_msg;
        rclcpp::Rate loop_rate(10);
        float error_distance = std::sqrt(pow(desired_pose.position.x,2)+pow(desired_pose.position.y,2));
        float orientation_angle = std::atan2(desired_pose.position.y, desired_pose.position.x);
        float error_orientation = abs(orientation_angle - (float)robot_yaw);
        RCLCPP_WARN(this->get_logger(), "ERROR DISTANCE TO FRONT SHELF ==> %f", error_distance);
        while (error_distance >= 0.1){
            
            vel_msg.linear.x = 0.2;
            vel_msg.angular.z = 0.0;
            vel_pub_->publish(vel_msg);

            //update pose
            error_distance = std::sqrt(pow(desired_pose.position.x,2)+pow(desired_pose.position.y,2));
            orientation_angle = std::atan2(desired_pose.position.y, desired_pose.position.x);
            error_orientation = abs(orientation_angle - (float)robot_yaw);
            RCLCPP_DEBUG(this->get_logger(), "ERROR DISTANCE TO FRONT SHELF ==> %f", error_distance);
            loop_rate.sleep();
        } 
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        vel_pub_->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "REACHED FRONT SHELF");
 
    }

    void move_under_shelf(){
    
        velocity vel_msg;

        vel_msg.linear.x = 0.2;
        vel_msg.angular.z = 0.0;
        vel_pub_->publish(vel_msg);
        usleep(5000000);

        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        vel_pub_->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "UNDER THE SHELF");
    
    }


    rclcpp::Service<go2loading>::SharedPtr srv_;
    rclcpp::Publisher<velocity>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lift_pub_;
    rclcpp::Subscription<laser>::SharedPtr laser_sub_;
    rclcpp::Subscription<odom>::SharedPtr odom_sub_;
    rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::CallbackGroup::SharedPtr srv_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::vector<float> intensity_array;
    pose odom_pose;
    pose desired_pose;
    double robot_yaw;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool found_shelf;
    float tx;
    float ty;
    
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto attach_server_node = std::make_shared<AttachShelfServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(attach_server_node);

    executor.spin();




    rclcpp::shutdown();
    return 0;
}




