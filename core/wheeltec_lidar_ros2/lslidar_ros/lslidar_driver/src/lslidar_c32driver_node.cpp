#include "lslidar_driver/lslidar_driver.h"
#include "std_msgs/msg/string.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler);
    lidar_type = "c32";

    // start the driver
    if ("c16" == lidar_type) {
        auto node = std::make_shared<lslidar_driver::lslidarC16Driver>();
        if (!node->initialize()) {
            RCLCPP_ERROR(node->get_logger(), "cannot initialize lslidar driver.");
            return 0;
        }
        while (rclcpp::ok()) {
            node->poll();
        }
        rclcpp::shutdown();

    } else {
        auto node = std::make_shared<lslidar_driver::lslidarC32Driver>();
        if (!node->initialize()) {
            RCLCPP_ERROR(node->get_logger(), "cannot initialize lslidar driver.");
            return 0;
        }
        while (rclcpp::ok()) {
            node->poll();
        }
        rclcpp::shutdown();
    }
    return 0;
}
