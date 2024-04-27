#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class ApproachColouredBoxAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:

  ApproachColouredBoxAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick()
  {
    getInput("box_pickup_position", goal_.pose);
  }

  BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("box_pickup_position", "box pickup position to approach"),
        });
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<ApproachColouredBoxAction>(
        name, "navigate_to_pose", config);
  };

  factory.registerBuilder<ApproachColouredBoxAction>(
      "ApproachColouredBox", builder);
}
