#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

std::string box_pickup_pose_pub_topic = "/box_pickup_pose";
std::string lidar_camera_fusion_sub_topic = "/lidar_camera_fusion";

const float pose_margin = 0.4;

class BoxPickupPosePublisher : public rclcpp::Node
{
  public:
    BoxPickupPosePublisher()
    : Node("box_pickup_pose_publisher"), canTransformBasefootprint(false),  tf_buffer_(this->get_clock())
    {
        lidar_camera_fusion_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(lidar_camera_fusion_sub_topic, 10, std::bind(&BoxPickupPosePublisher::lidar_camera_fusion_sub_topic_callback, this, _1));
        box_pickup_pose_pub =  this->create_publisher<geometry_msgs::msg::PoseStamped>(box_pickup_pose_pub_topic,10);
        tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        while(rclcpp::ok() && !canTransformBasefootprint)
        {
            if (tf_buffer_.canTransform("map", "base_footprint", tf2::TimePoint(), &warning_msg))
            {
                canTransformBasefootprint = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for transform %s ->  %s: %s", "map",
                            "base_footprint", warning_msg.c_str());
                std::this_thread::sleep_for(0.1s);
            }
        }

    }

  private:

    typedef struct {
        double r;       // a fraction between 0 and 1
        double g;       // a fraction between 0 and 1
        double b;       // a fraction between 0 and 1
    } rgb;

    typedef struct {
        double h;       // angle in degrees
        double s;       // a fraction between 0 and 1
        double v;       // a fraction between 0 and 1
    } hsv;

    hsv rgb2hsv(rgb in)
    {
        hsv         out;
        double      min, max, delta;

        min = in.r < in.g ? in.r : in.g;
        min = min  < in.b ? min  : in.b;

        max = in.r > in.g ? in.r : in.g;
        max = max  > in.b ? max  : in.b;

        out.v = max;                                // v
        delta = max - min;
        if (delta < 0.00001)
        {
            out.s = 0;
            out.h = 0; // undefined, maybe nan?
            return out;
        }
        if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
            out.s = (delta / max);                  // s
        } else {
            // if max is 0, then r = g = b = 0              
            // s = 0, h is undefined
            out.s = 0.0;
            out.h = NAN;                            // its now undefined
            return out;
        }
        if( in.r >= max )                           // > is bogus, just keeps compilor happy
            out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
        else
        if( in.g >= max )
            out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

        out.h *= 60.0;                              // degrees

        if( out.h < 0.0 )
            out.h += 360.0;

        return out;
    }
    void get_base_footprint_transform(geometry_msgs::msg::TransformStamped &base_footprint_transform)
    {
        bool is_transform_found = false;
        while (rclcpp::ok() && !is_transform_found)
        {
            try
            {
                base_footprint_transform = tf_buffer_.lookupTransform(
                    "map", "base_footprint",
                    tf2::TimePoint());
                is_transform_found = true;
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s to %stransform lookup unsuccessful", "map", "base_footprint");
                std::this_thread::sleep_for(0.05s);
            }
        }
    }
    
    float dist(const float &point_1_x, const float &point_1_y, const float &point_2_x, const float &point_2_y){
        return pow((pow((point_2_x - point_1_x), 2) + pow((point_2_y - point_1_y), 2)), 0.5);
    }

    void get_goal_pose(const float &centroid_x, const float &centroid_y, const float &bot_x, const float &bot_y, geometry_msgs::msg::Pose &goal_pose){
        float m_plus_n = dist(centroid_x, centroid_y, bot_x, bot_y);
        float m = pose_margin;
        float n = m_plus_n - m;
        goal_pose.position.x = (m * bot_x + n * centroid_x) / m_plus_n;
        goal_pose.position.y = (m * bot_y + n * centroid_y) / m_plus_n;
        float through_vector_x = centroid_x - bot_x;
        float through_vector_y = centroid_y - bot_y;

        float unit_through_vector_x = through_vector_x/ m_plus_n;
        float unit_through_vector_y = through_vector_y/ m_plus_n;
        float unit_through_vector_z = 0.0;

        float yaw = atan2(unit_through_vector_y, unit_through_vector_x);
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        RCLCPP_DEBUG(this->get_logger(), "yaw: %f", yaw);
        goal_pose.orientation.x = quat.x();
        goal_pose.orientation.y = quat.y();
        goal_pose.orientation.z = quat.z();
        goal_pose.orientation.w = quat.w();
    }

    void lidar_camera_fusion_sub_topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg){
        int points_size = _msg->data.size() / _msg->point_step;
        std::vector<std::vector<float>> red_line;
        for(int i=0; i< points_size; i++){
            std::vector<float> point_xyz(3);
            std::vector<uint32_t> point_rgb(3);
            memcpy(&point_xyz[0], &_msg->data[i*_msg->point_step + 0], sizeof(float));
            memcpy(&point_xyz[1], &_msg->data[i*_msg->point_step + 4], sizeof(float));
            memcpy(&point_xyz[2], &_msg->data[i*_msg->point_step + 8], sizeof(float));
            std::uint32_t rgb_uint32;
            memcpy(&rgb_uint32, &_msg->data[i*_msg->point_step + 12], sizeof(std::uint32_t));
            point_rgb[0] = (rgb_uint32 >> 16) & 0x0000ff;
            point_rgb[1] = (rgb_uint32 >> 8)  & 0x0000ff;
            point_rgb[2] = (rgb_uint32)       & 0x0000ff;

            rgb in = {(double)point_rgb[0]/255, (double)point_rgb[1]/255, (double)point_rgb[2]/255}; 
            hsv out = rgb2hsv(in);
            if((out.h < 10.0 || out.h >350.0) && out.s>0.5){
                red_line.push_back(point_xyz);
            }else{
                red_line.clear();
            }

            if(red_line.size()>2){
                // float sum_x = 0; 
                // float sum_y = 0;
                // for(auto red_point : red_line){
                //     sum_x += red_point[0];
                //     sum_y += red_point[1];
                // }

                // float centroid_x = sum_x/red_line.size();
                // float centroid_y = sum_y/red_line.size();

                // float centroid_x = red_line[red_line.size()/2][0];
                // float centroid_y = red_line[red_line.size()/2][1];
                geometry_msgs::msg::TransformStamped base_footprint_transform;
                get_base_footprint_transform(base_footprint_transform);
                float bot_x = base_footprint_transform.transform.translation.x;
                float bot_y = base_footprint_transform.transform.translation.y;
                
                float min_dist, min_dist_index;
                for(int i = 0; i< red_line.size(); i++){
                    float red_point_x = red_line[i][0];
                    float red_point_y = red_line[i][1];
                    if(i==0){
                        min_dist = dist(red_point_x, red_point_y, bot_x, bot_y);
                        min_dist_index= 0;
                    }else{
                        float temp_min_dist = dist(red_point_x, red_point_y, bot_x, bot_y);
                        if(temp_min_dist < min_dist){
                            min_dist = temp_min_dist;
                            min_dist_index = i;
                        }
                    }
                }

                geometry_msgs::msg::Pose goal_pose;
                get_goal_pose(red_line[min_dist_index][0], red_line[min_dist_index][1], bot_x, bot_y, goal_pose);
                geometry_msgs::msg::PoseStamped goal_pose_stamped;
                goal_pose_stamped.pose = goal_pose;
                RCLCPP_DEBUG(this->get_logger(),"publishing pose: {%f %f}", goal_pose.position.x, goal_pose.position.y);
                box_pickup_pose_pub->publish(goal_pose_stamped);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_camera_fusion_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr box_pickup_pose_pub;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    bool canTransformBasefootprint = false;
    std::string warning_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxPickupPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
