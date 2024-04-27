#include <functional>
#include <memory>
#include <thread>

#include "wheeltec_rrt_msg/action/pick_coloured_box.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "wheeltec_rrt_msg/srv/delete_shape.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class PickColouredBoxActionServer : public rclcpp::Node
{
public:
    using PickColouredBox = wheeltec_rrt_msg::action::PickColouredBox;
    using GoalHandlePickColouredBox = rclcpp_action::ServerGoalHandle<PickColouredBox>;

    explicit PickColouredBoxActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("pick_coloured_box_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<PickColouredBox>(
            this,
            "pick_coloured_box",
            std::bind(&PickColouredBoxActionServer::handle_goal, this, _1, _2),
            std::bind(&PickColouredBoxActionServer::handle_cancel, this, _1),
            std::bind(&PickColouredBoxActionServer::handle_accepted, this, _1));

        box_deleter_client = this->create_client<wheeltec_rrt_msg::srv::DeleteShape>("delete_shape");
    }

private:
    rclcpp_action::Server<PickColouredBox>::SharedPtr action_server_;
    rclcpp::Client<wheeltec_rrt_msg::srv::DeleteShape>::SharedPtr box_deleter_client;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PickColouredBox::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with box colour %s", goal->box_colour);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickColouredBox> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickColouredBox> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&PickColouredBoxActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePickColouredBox> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // Currently box colour is redundant as red colour is hard coded
        // const std::string box_colour = goal_handle->get_goal();
        auto result = std::make_shared<PickColouredBox::Result>();
        if(rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
            } else {
                RCLCPP_INFO(this->get_logger(), "picking coloured box...");
                auto request = std::make_shared<wheeltec_rrt_msg::srv::DeleteShape::Request>();
                request->shape_type = "red_box";
                while (!box_deleter_client->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
                }
                auto response = box_deleter_client->async_send_request(request);
                goal_handle->succeed(result);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickColouredBoxActionServer>());
    return 0;
}
