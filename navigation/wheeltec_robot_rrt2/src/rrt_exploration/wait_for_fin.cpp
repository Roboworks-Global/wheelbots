#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <string.h>
#include <iostream>
#include <std_msgs/msg/int8.hpp>
#include <stdlib.h>
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.h>
#include <stdio.h>
#include "utils.h"
#include "mtrand.h"
using std::placeholders::_1;

std::string rrt_flag = "rrt_flag";

int overmap_flag =0;
int rrt_success = 0;
int count = 0 ;
int if_start = 0;



class WaitForFin : public rclcpp::Node
{
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub; 
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr rrt_flag_pub; // 自主建图开启标志位发布者
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr overmap_flag_pub; //建图完成标志位发布者
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr back_pub; 


	public:
	    void Pubtopic();
	WaitForFin()
	: Node("WaitForFin")
	{

	back_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
	rrt_flag_pub = this->create_publisher<std_msgs::msg::Int8>("rrt_flag", 1);
	overmap_flag_pub = this->create_publisher<std_msgs::msg::Int8>("overmap_flag", 1);
	

        
       odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_combined", 10, std::bind(&WaitForFin::odom_Callback, this, _1));
      
        start_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 10, std::bind(&WaitForFin::start_Callback, this, _1));
      

    }
    private:

		void start_Callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
		{
			if_start++;
			printf("small pig\n");
		}
	
		void odom_Callback(const nav_msgs::msg::Odometry::SharedPtr msg)
		{
			if(if_start > 4)
			{
				if(msg->twist.twist.linear.x < 0.005 && msg->twist.twist.angular.z < 0.005)
				{       			printf("small pig2\n");
					count++;
				}
				else 
				{
					count = 0;
				}
			}
		}

};
void WaitForFin::Pubtopic()
{
	int waiting_time ;
	waiting_time =20;
	this->declare_parameter<int>("/wait_for_fin/waiting_time", waiting_time);
	this->get_parameter("/wait_for_fin/waiting_time", waiting_time);
	
	geometry_msgs::msg::PoseStamped origin;
	origin.header.stamp =rclcpp::Time(0);
	origin.header.frame_id = "map";
	origin.pose.position.x = 0;
	origin.pose.position.y = 0;
	origin.pose.position.z = 0;
	origin.pose.orientation.x = 0;
	origin.pose.orientation.y = 0;
	origin.pose.orientation.z = 0;
	origin.pose.orientation.w = 1;
	
	double rate = 1;    
	rclcpp::Rate loopRate(rate);
	
	while(rclcpp::ok())
	{
		if(count == waiting_time)
		{
			system("dbus-launch gnome-terminal -- ros2 launch wheeltec_nav2 save_map.launch.py");
			system("dbus-launch gnome-terminal -- rosnode kill /assigner");
		}
		
		if(count == (waiting_time+5))
		{
			back_pub->publish(origin);
			std_msgs::msg::Int8 overmap_flag_msg;
			overmap_flag_msg.data = 1;
			overmap_flag_pub->publish(overmap_flag_msg);//建图完成标志位发布
			printf("a=%d",overmap_flag_msg.data);
			break;
		}
	
		printf("count=%d\n",count);
		if(rrt_success == 0)
		{
		std_msgs::msg::Int8 rrt_flag_msg;
		rrt_flag_msg.data = 1;
		rrt_flag_pub->publish(rrt_flag_msg);//自主建图开启成功标志位发布
		printf("a=%d",rrt_flag_msg.data);
		rrt_success = 1;
		}
		rclcpp::spin_some(this->get_node_base_interface());
		loopRate.sleep();      
	}			
}

int main(int argc, char **argv)
{
	
	rclcpp::init(argc,argv);
	auto node = std::make_shared<rclcpp::Node>("wait_for_fin"); 

	WaitForFin Pub;
	Pub.Pubtopic();
	return 0;
}

