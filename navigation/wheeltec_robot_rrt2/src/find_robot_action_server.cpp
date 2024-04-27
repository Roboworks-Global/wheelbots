#include <functional>
#include <memory>
#include <thread>

#include "wheeltec_rrt_msg/action/find_coloured_box.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

std::string box_pickup_position_sub_topic = "/box_pickup_pose";
std::string assigned_frontier_goal_sub_topic = "/assigned_frontier_goal";


class FindColouredBoxActionServer : public rclcpp::Node
{
public:
  using FindColouredBox = wheeltec_rrt_msg::action::FindColouredBox;
  using GoalHandleFindColouredBox = rclcpp_action::ServerGoalHandle<FindColouredBox>;

  explicit FindColouredBoxActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("find_coloured_box_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FindColouredBox>(
      this,
      "find_coloured_box",
      std::bind(&FindColouredBoxActionServer::handle_goal, this, _1, _2),
      std::bind(&FindColouredBoxActionServer::handle_cancel, this, _1),
      std::bind(&FindColouredBoxActionServer::handle_accepted, this, _1));
    
    box_pickup_position_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      box_pickup_position_sub_topic, 10, std::bind(&FindColouredBoxActionServer::box_pickup_position_sub_topic_callback, this, _1));
    assigned_frontier_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      assigned_frontier_goal_sub_topic, 10, std::bind(&FindColouredBoxActionServer::assigned_frontier_goal_sub_topic_callback, this, _1));
    
    this->action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    this->action_client_->wait_for_action_server();
  }

private:

  rclcpp_action::Server<FindColouredBox>::SharedPtr action_server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr box_pickup_position_sub, assigned_frontier_goal_sub;
  geometry_msgs::msg::PoseStamped::SharedPtr box_pickup_position, assigned_frontier_goal;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client_;

  void box_pickup_position_sub_topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
  {
      RCLCPP_DEBUG(this->get_logger(), "recived box pickup pose");
      this->box_pickup_position = _msg;
  }

  void assigned_frontier_goal_sub_topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
  {
      RCLCPP_DEBUG(this->get_logger(), "recived box assigned frontier pose");
      this->assigned_frontier_goal = _msg;
  }

  
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FindColouredBox::Goal> goal)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request with box colour %s", goal->box_colour);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindColouredBox> goal_handle)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindColouredBox> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FindColouredBoxActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindColouredBox> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    // Currently box colour is redundant as red colour is hard coded
    // const std::string box_colour = goal_handle->get_goal();
    auto result = std::make_shared<FindColouredBox::Result>();
    for (int i = 1; rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }else if( this->box_pickup_position.use_count() != 0){
        result->box_pickup_position = *(this->box_pickup_position);
        goal_handle->succeed(result);
        this->box_pickup_position.reset();
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }else if(this->assigned_frontier_goal.use_count() != 0){
        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose = *(this->assigned_frontier_goal);
        this->assigned_frontier_goal.reset();
        action_client_->async_send_goal(goal);
      }
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindColouredBoxActionServer>());
    return 0;
}
