#include <rclcpp/rclcpp.hpp>
#include <termio.h>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>


class ArmKeyboardControlNode : public rclcpp::Node {
public:
    ArmKeyboardControlNode();

private:
    rclcpp::TimerBase::SharedPtr timer;
    struct termios new_settings;
    struct termios stored_settings;

    double Car_foward_velocity;
    double Car_lateral_velocity;
    double Car_turn_velocity;

    bool Omni = false; 
    int key_wait_count=0;
    bool cmd_vel_enable=1;
    int Programe_terminate=0;

    int scan_keyboard();
    void Threading_key_scan();
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

    std::thread key_scan_thread;
};