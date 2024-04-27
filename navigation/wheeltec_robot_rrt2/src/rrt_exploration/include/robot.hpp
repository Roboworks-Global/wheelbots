// #include "move_base_msgs/MoveBaseGoal.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rrt_exploration
{

    class Robot
    {
    public:
        // explicit Robot(std::string name)
        Robot();

        void send_goal(const geometry_msgs::msg::Point &);

        std::vector<double> get_position();

        std::vector<double> get_assigned_point();
    private:
        // std::string name;
        rclcpp::Node::SharedPtr nodePtr;
        std::string global_frame, robot_frame, planner_service;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        // std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client_;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;
        std::vector<double> assigned_point;
    };
}