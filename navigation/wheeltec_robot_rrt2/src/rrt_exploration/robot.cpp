// #include "move_base_msgs/MoveBaseGoal.h"
#include "robot.hpp"

namespace rrt_exploration{
// explicit Robot(std::string name)
Robot::Robot()
{
    nodePtr = std::make_shared<rclcpp::Node>("sandwich_bot_2_0_robot");
//   this->name = name;
    nodePtr->declare_parameter<std::string>("global_frame", "map");
    nodePtr->declare_parameter<std::string>("robot_frame", "base_footprint");

    nodePtr->get_parameter("global_frame", this->global_frame);
    nodePtr->get_parameter("robot_frame", this->robot_frame);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nodePtr->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // std::string source_frame_id = this->name + "/" + robot_frame;
    std::string source_frame_id = robot_frame;

    std::string target_frame_id = global_frame;
    std::string warning_msg;
    rclcpp::Rate wait_rate(10);
    while (rclcpp::ok() && !tf_buffer_->canTransform(
                                source_frame_id, target_frame_id, tf2::TimePoint(), &warning_msg))
    {
    RCLCPP_INFO(
        nodePtr->get_logger(), "Waiting for transform %s ->  %s: %s",
        source_frame_id.c_str(), target_frame_id.c_str(), warning_msg.c_str());
        wait_rate.sleep();
        // std::this_thread::sleep_for(std::chrono::nanoseconds(rclcpp::Duration(0.1).nanoseconds()));
    }      

    // action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(nodePtr, "navigate_to_pose");
    goal_pose_pub= nodePtr->create_publisher<geometry_msgs::msg::PoseStamped>("/assigned_frontier_goal",10);

    this->assigned_point = this->get_position();
    
    // Make sure the server is actually there before continuing
    RCLCPP_INFO(nodePtr->get_logger(), "Waiting for \"%s\" action server", "navigate_to_pose");
    // action_client_->wait_for_action_server();

    goal_pose.header.frame_id = this->global_frame;
}

void Robot::send_goal(const geometry_msgs::msg::Point &goal_pose_unstamped){
    goal_pose.header.stamp = nodePtr->get_clock()->now();
    goal_pose.pose.position = goal_pose_unstamped;
    goal_pose.pose.orientation.w = 1.0;
    // nav2_msgs::action::NavigateToPose::Goal goal;
    // goal.pose = goal_pose;
    // action_client_->async_send_goal(goal);
    goal_pose_pub->publish(goal_pose);
}

std::vector<double> Robot::get_position(){
    geometry_msgs::msg::TransformStamped transformStamped;
    // transformStamped = tf_buffer_->lookupTransform(global_frame, this->name + "/" + robot_frame, tf2::TimePoint());
    transformStamped = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePoint());
    return {transformStamped.transform.translation.x, transformStamped.transform.translation.y};
}

std::vector<double> Robot::get_assigned_point(){
    return assigned_point;
}
}