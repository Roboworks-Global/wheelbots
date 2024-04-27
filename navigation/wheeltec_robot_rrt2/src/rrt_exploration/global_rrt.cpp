#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "utils.h"
#include "mtrand.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string map_topic = "/map";
std::string rviz_sub_topic = "/clicked_point";
std::string targets_pub_topic = "/detected_frontiers";
std::string marker_pub_topic = "/_shapes";

std::string map_frame_id = "map";
double eta =  0.3;

//// namespace required?

class GlobalRRT : public rclcpp::Node
{
  public:
    GlobalRRT()
    : Node("global_rrt"), are_boundary_points_receieved(false)
    {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, 100, std::bind(&GlobalRRT::map_topic_callback, this, _1));

        rviz_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        rviz_sub_topic, 100, std::bind(&GlobalRRT::rviz_topic_callback, this, _1));
    
        targets_pub =  this->create_publisher<geometry_msgs::msg::PointStamped>(targets_pub_topic,10);
        marker_pub =  this->create_publisher<visualization_msgs::msg::Marker>(marker_pub_topic,10);
    
        timer_  = rclcpp::create_timer(this, rclcpp::Clock::make_shared(), 0.05s,
                      std::bind(&GlobalRRT::timer_callback, this));

        boundary_points.header.frame_id = map_frame_id;
        boundary_points.header.stamp = rclcpp::Time(0);
        boundary_points.ns = "markers";
        boundary_points.id = 3;
        boundary_points.type = boundary_points.POINTS;
        boundary_points.action = boundary_points.ADD;
        boundary_points.pose.orientation.w = 1.0;
        boundary_points.scale.x = 0.3;
        boundary_points.scale.y = 0.3;
        boundary_points.color.r = 255.0 / 255.0;
        boundary_points.color.g = 0.0 / 255.0;
        boundary_points.color.b = 0.0 / 255.0;
        boundary_points.color.a = 0.3;

        //visualizations  points and lines..
        frontier_points_rviz.header.frame_id = map_frame_id;
        frontier_points_rviz.header.stamp = rclcpp::Time(0);
        frontier_points_rviz.ns = "markers";
        frontier_points_rviz.id = 4;
        frontier_points_rviz.type = frontier_points_rviz.POINTS;
        frontier_points_rviz.action = frontier_points_rviz.ADD;
        frontier_points_rviz.pose.orientation.w = 1.0;
        frontier_points_rviz.scale.x = 0.3;
        frontier_points_rviz.scale.y = 0.3;
        frontier_points_rviz.color.r = 255.0 / 255.0;
        frontier_points_rviz.color.g = 0.0 / 255.0;
        frontier_points_rviz.color.b = 0.0 / 255.0;
        frontier_points_rviz.color.a = 1.0;
        // points.lifetime = rclcpp::Duration();
        
        rrt_lines_rviz.header.frame_id = map_frame_id;
        rrt_lines_rviz.header.stamp = rclcpp::Time(0);
        rrt_lines_rviz.ns = "markers";
        rrt_lines_rviz.id = 5;
        rrt_lines_rviz.type = rrt_lines_rviz.LINE_LIST;
        rrt_lines_rviz.action = rrt_lines_rviz.ADD;
        rrt_lines_rviz.scale.x = 0.03;
        rrt_lines_rviz.pose.orientation.w = 1.0;
        rrt_lines_rviz.scale.y = 0.03;
        rrt_lines_rviz.color.r = 0.0 / 255.0;
        rrt_lines_rviz.color.g = 0.0 / 255.0;
        rrt_lines_rviz.color.b = 255.0 / 255.0;
        rrt_lines_rviz.color.a = 1.0;
        // line.lifetime = rclcpp::Duration();
    
    }

  private:
    
    void map_topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        mapData = *msg;
    }

    void rviz_topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if(are_boundary_points_receieved){
            return;
        }

        geometry_msgs::msg::Point p;
        p.x = msg->point.x;
        p.y = msg->point.y;
        p.z = msg->point.z;

        boundary_points.points.push_back(p);
        
        if( boundary_points.points.size() == 5){
            init_map_x = norm({boundary_points.points[0].x, boundary_points.points[0].y}, {boundary_points.points[2].x, boundary_points.points[0].y});
            init_map_y = norm({boundary_points.points[0].x, boundary_points.points[0].y}, {boundary_points.points[0].x, boundary_points.points[2].y});

            Xstartx = (boundary_points.points[0].x + boundary_points.points[2].x) * .5;
            Xstarty = (boundary_points.points[0].y + boundary_points.points[2].y) * .5;

            V.push_back({boundary_points.points[4].x, boundary_points.points[4].y});

            boundary_points.points.clear();
            are_boundary_points_receieved = true;
        }
        marker_pub->publish(boundary_points);
    }

    void timer_callback(){
        if(mapData.data.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for the map");
            return;
        }

        if(!are_boundary_points_receieved){
            return;
        }

        std::vector<double> x_rand, x_nearest, x_new;
        double xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
        double yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;
        x_rand= {xr, yr};
        x_nearest = Nearest(V, x_rand);
        x_new = Steer(x_nearest, x_rand, eta);

        // ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
        char checking = ObstacleFree(x_nearest, x_new, mapData);
        //std::cout<<"chacking is "<<checking<<std::endl;

        if (checking == 255)
        {
            exploration_goal.header.stamp = this->get_clock()->now();
            exploration_goal.header.frame_id = mapData.header.frame_id;
            exploration_goal.point.x = x_new[0];
            exploration_goal.point.y = x_new[1];
            exploration_goal.point.z = 0.0;
          	targets_pub->publish(exploration_goal);
          	//RCLCPP_INFO(this->get_logger(), "\x1b[32m" "\33[1m%s\33[0m" "\x1b[0m", "exploration_goal");
            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            frontier_points_rviz.points.push_back(p);
          	marker_pub->publish(frontier_points_rviz) ;
		  	frontier_points_rviz.points.clear();

            // is push_back necessary?
            //V.push_back({transform.transform.translation.x, transform.transform.translation.y});
        }

        else if (checking == 1)
        {
            V.push_back(x_new);
            
            geometry_msgs::msg::Point p;
            
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            rrt_lines_rviz.points.push_back(p);

            p.x = x_nearest[0];
            p.y = x_nearest[1];
            p.z = 0.0;
            rrt_lines_rviz.points.push_back(p);
        }

        marker_pub->publish(rrt_lines_rviz);
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_subscription_;
    nav_msgs::msg::OccupancyGrid mapData;
    geometry_msgs::msg::PointStamped clickedpoint;
    geometry_msgs::msg::PointStamped exploration_goal;
    visualization_msgs::msg::Marker boundary_points, frontier_points_rviz, rrt_lines_rviz;
    double xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y, range;
    unsigned long mtrand_init[4] = {0x123, 0x234, 0x345, 0x456} ;
    MTRand drand;                     // double in [0, 1) generator, already init
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr targets_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> V;
    bool are_boundary_points_receieved;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GlobalRRT>());

    rclcpp::shutdown();
    return 0;
}
