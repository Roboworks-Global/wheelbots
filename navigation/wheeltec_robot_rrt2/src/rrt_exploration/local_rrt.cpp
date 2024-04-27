#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "utils.h"
#include "mtrand.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

std::string map_topic = "/map";
std::string rviz_sub_topic = "/clicked_point";
std::string detected_frontiers_pub_topic = "/detected_frontiers";
std::string marker_pub_topic = "/_shapes";

std::string map_frame_id = "map";
std::string robot_base_frame_id = "base_footprint";
double eta =  0.3;

//// namespace required?

class LocalRRT : public rclcpp::Node
{
  public:
    LocalRRT()

    : Node("local_rrt"), canTransform(false), tf_buffer_(this->get_clock()), are_boundary_points_receieved(false)
    {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, 100, std::bind(&LocalRRT::map_topic_callback, this, _1));

        rviz_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        rviz_sub_topic, 100, std::bind(&LocalRRT::rviz_topic_callback, this, _1));

        detected_frontiers_pub =  this->create_publisher<geometry_msgs::msg::PointStamped>(detected_frontiers_pub_topic,10);
        marker_pub =  this->create_publisher<visualization_msgs::msg::Marker>(marker_pub_topic,10);
        
        timer_  = rclcpp::create_timer(this, rclcpp::Clock::make_shared(), 0.05s,
                      std::bind(&LocalRRT::timer_callback, this));

        //visualizations  points and lines..
        frontier_points_rviz.header.frame_id = map_frame_id;
        frontier_points_rviz.header.stamp = rclcpp::Time(0);
        frontier_points_rviz.ns = "markers";
        frontier_points_rviz.id = 1;
        frontier_points_rviz.type = frontier_points_rviz.POINTS;
        frontier_points_rviz.action = frontier_points_rviz.ADD;
        frontier_points_rviz.pose.orientation.w = 1.0;
        frontier_points_rviz.scale.x = 0.3;
        frontier_points_rviz.scale.y = 0.3;
        frontier_points_rviz.color.r = 255.0 / 255.0;
        frontier_points_rviz.color.g = 0.0 / 255.0;
        frontier_points_rviz.color.b = 0.0 / 255.0;
        frontier_points_rviz.color.a = 0.3;
        // points.lifetime = rclcpp::Duration();
        
        rrt_lines_rviz.header.frame_id = map_frame_id;
        rrt_lines_rviz.header.stamp = rclcpp::Time(0);
        rrt_lines_rviz.ns = "markers";
        rrt_lines_rviz.id = 2;
        rrt_lines_rviz.type = rrt_lines_rviz.LINE_LIST;
        rrt_lines_rviz.action = rrt_lines_rviz.ADD;
        rrt_lines_rviz.scale.x = 0.03;
        rrt_lines_rviz.pose.orientation.w = 1.0;
        rrt_lines_rviz.scale.y = 0.03;
        rrt_lines_rviz.color.r = 255.0 / 255.0;
        rrt_lines_rviz.color.g = 0.0 / 255.0;
        rrt_lines_rviz.color.b = 0.0 / 255.0;
        rrt_lines_rviz.color.a = 1.0;
        // line.lifetime = rclcpp::Duration();
    

        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
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

        if( boundary_points.points.size()==5){
            init_map_x = norm({boundary_points.points[0].x, boundary_points.points[0].y}, {boundary_points.points[2].x, boundary_points.points[0].y});
            init_map_y = norm({boundary_points.points[0].x, boundary_points.points[0].y}, {boundary_points.points[0].x, boundary_points.points[2].y});

            Xstartx = (boundary_points.points[0].x + boundary_points.points[2].x) * .5;
            Xstarty = (boundary_points.points[0].y + boundary_points.points[2].y) * .5;
            
            V.push_back({boundary_points.points[4].x, boundary_points.points[4].y});
            are_boundary_points_receieved = true;
        }
    }

    void timer_callback(){
        if(mapData.data.empty()){
            RCLCPP_INFO(this->get_logger(),"Waiting for the map");
            return;
        }

        if(!are_boundary_points_receieved){
            return;
        }

        if(!canTransform){
            if(tf_buffer_.canTransform(map_frame_id, robot_base_frame_id, tf2::TimePoint(), &warning_msg))
            {
                canTransform = true;
            }else
            {
                RCLCPP_INFO(this->get_logger(),"Waiting for transform %s ->  %s: %s",
                    map_frame_id.c_str(), robot_base_frame_id.c_str(), warning_msg.c_str());
                return;
            }
        }

        std::vector<double> x_rand, x_nearest, x_new;
        double xr = (drand() * init_map_x) - (init_map_x * 0.5) + Xstartx;
        double yr = (drand() * init_map_y) - (init_map_y * 0.5) + Xstarty;
        x_rand= {xr, yr};
        x_nearest = Nearest(V, x_rand);
        x_new = Steer(x_nearest, x_rand, eta);

        // ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
        char checking = ObstacleFree(x_nearest, x_new, mapData);
        if (checking == 255)
        {
            exploration_goal.header.stamp = this->get_clock()->now();
            exploration_goal.header.frame_id = mapData.header.frame_id;
            exploration_goal.point.x = x_new[0];
            exploration_goal.point.y = x_new[1];
            exploration_goal.point.z = 0.0;
          	detected_frontiers_pub->publish(exploration_goal);

            geometry_msgs::msg::Point p;
            p.x = x_new[0];
            p.y = x_new[1];
            p.z = 0.0;
            frontier_points_rviz.points.push_back(p);
          	marker_pub->publish(frontier_points_rviz) ;
		  	frontier_points_rviz.points.clear();
		  	
            V.clear();
			geometry_msgs::msg::TransformStamped transform;
			bool is_transform_found = false;
            while (!is_transform_found)
            {
                try
                {
                    is_transform_found = true;
                    transform = tf_buffer_.lookupTransform(
                                        map_frame_id, robot_base_frame_id,
                                        ////is this valid time?
                                        tf2::TimePoint());
                }
                catch (tf2::TransformException ex)
                {
                    is_transform_found = false;
                    RCLCPP_ERROR(this->get_logger(),"transform lookup unsuccessful");
                    std::this_thread::sleep_for(0.1s);
                }
            }

            V.push_back({transform.transform.translation.x, transform.transform.translation.y});
            rrt_lines_rviz.points.clear();
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
    MTRand drand;                     // double in [0, 1) generator, already init
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detected_frontiers_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    bool canTransform;
    std::vector<std::vector<double>> V ;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::string warning_msg;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid mapData;
    geometry_msgs::msg::PointStamped clickedpoint;
    geometry_msgs::msg::PointStamped exploration_goal;
    visualization_msgs::msg::Marker boundary_points, frontier_points_rviz, rrt_lines_rviz;
    double xdim, ydim, resolution, Xstartx, Xstarty, init_map_x, init_map_y, range;
    unsigned long mtrand_init[4] = {0x123, 0x234, 0x345, 0x456};
    bool are_boundary_points_receieved;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalRRT>());
    rclcpp::shutdown();
    return 0;
}
