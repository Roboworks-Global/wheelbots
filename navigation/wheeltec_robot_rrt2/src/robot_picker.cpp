#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
std::chrono::milliseconds bt_loop_duration_;
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("box_picker");

  std::string bt_xml_filename;
  int timeout;
  node->declare_parameter<std::string>("bt_xml_filename", "");
  node->get_parameter("bt_xml_filename", bt_xml_filename);
  node->get_parameter("bt_loop_duration", timeout); 
  bt_loop_duration_ = std::chrono::milliseconds(1000);
  
  BT::Tree tree_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  BT::Blackboard::Ptr blackboard_;
  const std::vector<std::string> plugin_libs = {
      "find_coloured_box",
      "approach_coloured_box",
      "pick_coloured_box"};

  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libs);

  blackboard_ = BT::Blackboard::create();
  blackboard_->set<std::string>("box_colour", "red_box");
  blackboard_->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000)); 
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);

  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good())
  {
    RCLCPP_ERROR(node->get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }
  auto xml_string = std::string(
      std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());
  tree_ = bt_->createTreeFromText(xml_string, blackboard_);

  auto is_canceling = []()
  {
    return !rclcpp::ok();
  };
  auto on_loop = [&]() {

  };

  nav2_behavior_tree::BtStatus rc;
  do{
   rc = bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
    bt_->haltAllActions(tree_.rootNode());

  }while(rc == nav2_behavior_tree::BtStatus::SUCCEEDED);


  switch (rc)
  {
    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(node->get_logger(), "box picking  failed");
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(node->get_logger(), "box picking  canceled");
      break;
  }

  return 0;
}
