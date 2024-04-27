#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "wheeltec_rrt_msg/action/find_coloured_box.hpp"

class FindColouredBoxAction : public nav2_behavior_tree::BtActionNode<wheeltec_rrt_msg::action::FindColouredBox>
{
public:

  FindColouredBoxAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<wheeltec_rrt_msg::action::FindColouredBox>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick()
  {
    getInput("box_colour", goal_.box_colour);
  }

  BT::NodeStatus on_success()
  {
    setOutput("box_pickup_position", result_.result->box_pickup_position);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        {
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("box_pickup_position", "pickup position of the box found"),
            BT::InputPort<std::string>("box_colour", "colour of the box to seach for"),
        });
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<FindColouredBoxAction>(
        name, "find_coloured_box", config);
  };

  factory.registerBuilder<FindColouredBoxAction>(
      "FindColouredBox", builder);
}
