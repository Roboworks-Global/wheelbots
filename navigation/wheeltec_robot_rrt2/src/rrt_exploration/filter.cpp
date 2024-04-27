#include "rclcpp/rclcpp.hpp"
#include "utils.h"
#include "mean_shift.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "wheeltec_rrt_msg/msg/point_array.hpp"

std::string rviz_unfiltered_centroids_publisher_topic = "/unfiltered_centroids";
std::string rviz_filtered_centroids_publisher_topic = "/filtered_centroids";
std::string filtered_goal_points_publisher_topic = "/filtered_goal_points";

std::string map_frame_id = "map";

using std::placeholders::_1;
using namespace std::chrono_literals;

class Filter : public rclcpp::Node
{
  public:
    Filter()
    : Node("filter")
    {
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<int>("costmap_clearing_threshold", 70);
        this->declare_parameter<double>("info_radius", 1.0);
        this->declare_parameter<std::string>("goals_topic", "/detected_frontiers");
        this->declare_parameter<std::string>("global_costmap_topic", "/global_costmap/costmap");

        this->get_parameter("map_topic", map_topic);
        this->get_parameter("costmap_clearing_threshold", threshold);
        this->get_parameter("info_radius", info_radius);
        this->get_parameter("goals_topic", goals_topic);
        this->get_parameter("global_costmap_topic", global_costmap_topic);

        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, rclcpp::QoS(10), std::bind(&Filter::map_topic_callback, this, _1));

        global_map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            global_costmap_topic, rclcpp::QoS(10), std::bind(&Filter::global_map_topic_callback, this, _1));

        frontiers_subscription_ =  this->create_subscription<geometry_msgs::msg::PointStamped>(
            goals_topic, rclcpp::QoS(10), std::bind(&Filter::goals_topic_callback, this, _1));

        timer_  = rclcpp::create_timer(this, rclcpp::Clock::make_shared(), 0.1s,
                      std::bind(&Filter::timer_callback, this));


        rviz_unfiltered_centroids_publisher = this->create_publisher<visualization_msgs::msg::Marker>(rviz_unfiltered_centroids_publisher_topic,10);
        rviz_filtered_centroids_publisher = this->create_publisher<visualization_msgs::msg::Marker>(rviz_filtered_centroids_publisher_topic,10);
        filtered_goal_points_publisher = this->create_publisher<wheeltec_rrt_msg::msg::PointArray>(filtered_goal_points_publisher_topic,10);

        unfiltered_centroid_points_rviz.header.frame_id = map_frame_id;
        unfiltered_centroid_points_rviz.header.stamp = this->now();
        unfiltered_centroid_points_rviz.ns = "markers2";
        unfiltered_centroid_points_rviz.id = 0;
        unfiltered_centroid_points_rviz.type = unfiltered_centroid_points_rviz.POINTS;
        unfiltered_centroid_points_rviz.action = unfiltered_centroid_points_rviz.ADD;
        unfiltered_centroid_points_rviz.pose.orientation.w = 1.0;
        unfiltered_centroid_points_rviz.scale.x = 0.2;
        unfiltered_centroid_points_rviz.scale.y = 0.2;
        unfiltered_centroid_points_rviz.color.r = 255.0 / 255.0;
        unfiltered_centroid_points_rviz.color.g = 255.0 / 255.0;
        unfiltered_centroid_points_rviz.color.b = 0.0   / 255.0;
        unfiltered_centroid_points_rviz.color.a = 1.0;
    // unfiltered_centroid_points_rviz.lifetime = rclcpp::Duration();

        filtered_centroid_points_rviz.header.frame_id = map_frame_id;
        filtered_centroid_points_rviz.header.stamp = this->now();
        filtered_centroid_points_rviz.ns = "markers3";
        filtered_centroid_points_rviz.id = 4;
        filtered_centroid_points_rviz.type = filtered_centroid_points_rviz.POINTS;
        filtered_centroid_points_rviz.action = filtered_centroid_points_rviz.ADD;
        filtered_centroid_points_rviz.pose.orientation.w = 1.0;
        filtered_centroid_points_rviz.scale.x = 0.2;
        filtered_centroid_points_rviz.scale.y = 0.2;
        filtered_centroid_points_rviz.color.r = 0.0/255.0;
        filtered_centroid_points_rviz.color.g = 255.0/255.0;
        filtered_centroid_points_rviz.color.b = 0.0/255.0;
        filtered_centroid_points_rviz.color.a = 1.0;
    // filtered_centroid_points_rviz.lifetime = rclcpp::Duration();
    }

  private:
    
    void map_topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        mapData = *msg;
    }
    
    void global_map_topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        global_map = *msg;
    }

    void goals_topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        frontiers.push_back(*msg);
    }

    void timer_callback(){
        if(mapData.data.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for the map");
            return;
        }
        if(global_map.data.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for the global map");
            return;
        }
        if(frontiers.empty()){
            RCLCPP_DEBUG(this->get_logger(),"Waiting for the frontiers...");
            return;
        }

        std::vector<std::vector<double>> frontiers_vector = {};
        std::vector<std::vector<double>> unfiltered_centroids = {};

        auto frontiers_copy = frontiers;
        frontiers.clear();
        if(frontiers_copy.size() == 1){
            unfiltered_centroids.push_back({frontiers_copy[0].point.x, frontiers_copy[0].point.y});
        }else{
            for(auto frontier: frontiers_copy){
                frontiers_vector.push_back({frontier.point.x , frontier.point.y});
            }
            std::vector<Cluster> cluster_v = mean_shift.cluster(frontiers_vector, 0.3);
            for(auto cluster: cluster_v){
                unfiltered_centroids.push_back(cluster.mode);
            }
        }

        std::vector<std::vector<double>> filtered_centroids = {};
        
        for(auto centroid: unfiltered_centroids){
            bool condition = grid_value(global_map, {centroid[0], centroid[1]}) > threshold;
            bool around = (checkAround(mapData,{centroid[0], centroid[1]},info_radius*0.5)>1000);
            if(!(condition || information_gain(mapData, {centroid[0], centroid[1]}, info_radius*0.5) < 0.05)){
                filtered_centroids.push_back(centroid);
            }
        }
        // RCLCPP_INFO(this->get_logger(),">>unfiltered centroids size:%d",unfiltered_centroids.size());
        // RCLCPP_INFO(this->get_logger(),">>filtered centroids size:%d",filtered_centroids.size());

        wheeltec_rrt_msg::msg::PointArray filtered_goal_points;

        filtered_goal_points.points = {};
        for(auto filtered_centroid: filtered_centroids){
            geometry_msgs::msg::Point p;
            p.x=filtered_centroid[0];
            p.y=filtered_centroid[1];
            p.z = 0.0;
            filtered_goal_points.points.push_back(p);
            filtered_centroid_points_rviz.points.push_back(p);
        }

        for(auto unfiltered_centroid: unfiltered_centroids){
            geometry_msgs::msg::Point p;
            p.x=unfiltered_centroid[0];
            p.y=unfiltered_centroid[1];
            p.z = 0.0;
            unfiltered_centroid_points_rviz.points.push_back(p);
        }

        filtered_goal_points_publisher->publish(filtered_goal_points);
        rviz_unfiltered_centroids_publisher->publish(unfiltered_centroid_points_rviz);
        rviz_filtered_centroids_publisher->publish(filtered_centroid_points_rviz);
        unfiltered_centroid_points_rviz.points.clear();
        filtered_centroid_points_rviz.points.clear();
    }

    std::string map_topic, goals_topic, global_costmap_topic;
    int threshold; 
    double info_radius;
    nav_msgs::msg::OccupancyGrid mapData, global_map;
    std::vector<geometry_msgs::msg::PointStamped> frontiers;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr frontiers_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_unfiltered_centroids_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_filtered_centroids_publisher;
    rclcpp::Publisher<wheeltec_rrt_msg::msg::PointArray>::SharedPtr filtered_goal_points_publisher;
    visualization_msgs::msg::Marker unfiltered_centroid_points_rviz, filtered_centroid_points_rviz;
    MeanShift mean_shift;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Filter>());
    rclcpp::shutdown();
    return 0;
}
