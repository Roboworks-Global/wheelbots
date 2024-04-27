#include "rclcpp/rclcpp.hpp"
#include "utils.h"

#include "wheeltec_rrt_msg/msg/point_array.hpp"
#include "robot.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Assigner : public rclcpp::Node
{
  public:
    Assigner()
    : Node("assigner")
    {
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<double>("info_radius", 1.0);
        this->declare_parameter<double>("info_multiplier", 3.0);
        this->declare_parameter<double>("hysteresis_radius", 3.0);
        this->declare_parameter<double>("hysteresis_gain", 2.0);
        this->declare_parameter<std::string>("frontiers_topic", "/filtered_goal_points");
        
        this->get_parameter("map_topic", map_topic);
        this->get_parameter("info_radius", info_radius);
        this->get_parameter("info_multiplier", info_multiplier);
        this->get_parameter("hysteresis_radius", hysteresis_radius);
        this->get_parameter("hysteresis_gain", hysteresis_gain);
        this->get_parameter("frontiers_topic", frontiers_topic);

        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, 100, std::bind(&Assigner::map_topic_callback, this, _1));

        frontiers_subscription_ = this->create_subscription<wheeltec_rrt_msg::msg::PointArray>(
        frontiers_topic, 10, std::bind(&Assigner::frontiers_topic_callback, this, _1));

        timer_  = rclcpp::create_timer(this, rclcpp::Clock::make_shared(), 3s,
                      std::bind(&Assigner::timer_callback, this));

    }

  private:
    
    void map_topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        mapData = *msg;
    }

    void frontiers_topic_callback(const wheeltec_rrt_msg::msg::PointArray::SharedPtr msg)
    {
        frontiers = msg->points;
    }

    void timer_callback(){
        if(mapData.data.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for the map");
            return;
        }
        if(frontiers.empty()){
            RCLCPP_DEBUG(this->get_logger(),"Waiting for the filtered frontiers...");
            return;
        }

        std::vector<double> revenue_record = {};
        auto frontiers_copy = frontiers;
        for (auto frontier : frontiers_copy)
        {
            auto cost = norm(robot.get_position(), {frontier.x, frontier.y});
            double info_gain = information_gain(mapData, {frontier.x, frontier.y}, info_radius);
            info_gain -= discount(mapData, robot.get_assigned_point(), {frontier.x, frontier.y}, info_radius);
            if (norm(robot.get_position(), {frontier.x, frontier.y}) <= hysteresis_radius){
                info_gain *= hysteresis_gain;
            }

            if (norm({frontier.x, frontier.y}, robot.get_assigned_point()) < hysteresis_radius){
                info_gain = information_gain(mapData, {frontier.x, frontier.y}, info_radius) *hysteresis_gain;
            }

            double revenue = info_gain * info_multiplier - cost;
            revenue_record.push_back(revenue);
        }

        int max_revenue_index = std::max_element(revenue_record.begin(),revenue_record.end()) - revenue_record.begin();
        RCLCPP_DEBUG(this->get_logger(), "sending goal pose: %f, %f",frontiers_copy[max_revenue_index].x, frontiers_copy[max_revenue_index].y);
        robot.send_goal(frontiers_copy[max_revenue_index]);
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<wheeltec_rrt_msg::msg::PointArray>::SharedPtr frontiers_subscription_;
    std::string map_topic, frontiers_topic;
    double info_radius, info_multiplier, hysteresis_radius, hysteresis_gain;
    nav_msgs::msg::OccupancyGrid mapData;
    std::vector<geometry_msgs::msg::Point> frontiers;
    rclcpp::TimerBase::SharedPtr timer_;
    rrt_exploration::Robot robot;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Assigner>());
    rclcpp::shutdown();
    return 0;
} 
