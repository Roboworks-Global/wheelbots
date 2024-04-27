#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "wheeltec_rrt_msg/action/pick_coloured_box.hpp"

class PickColouredBoxAction : public nav2_behavior_tree::BtActionNode<wheeltec_rrt_msg::action::PickColouredBox>
{
public:

  PickColouredBoxAction(
      const std::string &xml_tag_name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : BtActionNode<wheeltec_rrt_msg::action::PickColouredBox>(xml_tag_name, action_name, conf)
  {
  }

  void on_tick()
  {
    getInput("box_colour", goal_.box_colour);
  }

  BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        {
            BT::InputPort<std::string>("box_colour", "colour of the box to pick up"),
        });
  }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<PickColouredBoxAction>(
        name, "pick_coloured_box", config);
  };

  factory.registerBuilder<PickColouredBoxAction>(
      "PickColouredBox", builder);
}
