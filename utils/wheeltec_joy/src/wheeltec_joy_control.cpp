#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include<iostream>
using std::placeholders::_1;
using namespace std;
class wheeltec_joy : public rclcpp::Node
{
	
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    std_msgs::msg::Float64 vlinear_x; //默认值
    std_msgs::msg::Float64 vlinear_z;
public:
    wheeltec_joy() 
: Node ("wheeltec_joy")
{
    RCLCPP_INFO(this->get_logger(),"wheeltec_joy111");
   //读取参数服务器中的变量值
     flag_mec=0;
    
   this->declare_parameter<int>("axis_linear", 1);//默认axes[1]接收速度
   this->declare_parameter<int>("axis_angular", 0);   //默认axes[0]接收角度
   this->declare_parameter<double>("vlinear", 0.3); //默认线速度0.3 m/s  
   this->declare_parameter<double>("vangular", 1);// 默认角速度1 单位rad/s
 
   this->get_parameter("axis_linear", axis_linear);
   this->get_parameter("axis_angular", axis_angular);
   this->get_parameter("v_linear", v_linear);
   this->get_parameter("v_angular", v_angular);
         
    pub =  this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub =  this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&wheeltec_joy::callback, this, _1)); //订阅手柄发来的数据
 
} 


private:
    //void callback(const sensor_msgs::msg::Joy::SharedPtr Joy);
    void callback(const sensor_msgs::msg::Joy::SharedPtr Joy)   //键值回调函数
	 {
	   double vangle_key,linera_key;
	   double acce_x,acce_z;
	   geometry_msgs::msg::Twist v;
	   vangle_key =Joy->axes[axis_angular];  //获取axes[0]的值
	   linera_key =Joy->axes[axis_linear];  //获取axes[1]的值
	
	   acce_x=Joy->axes[4]+1.0;   //读取右摇杆的值对机器人的线速度进行加减速处理
	   acce_z=Joy->axes[3]+1.0;   //读取右摇杆的值对机器人的角速度进行加减速处理
	   //判断前进后退   
	   if(linera_key>0) 
	   {
	       dir=1;
	       vlinear_x.data=v_linear;
	   } 
	   else if(linera_key<0)
	   {
	      dir=-1;
	      vlinear_x.data=-v_linear;
	   }
	   else  
	   {
	    dir=1;
	    vlinear_x.data=0;
	   }
	   //判断左转右转，大于0为左转，小于0为右转
	   if(vangle_key>0)       vlinear_z.data=v_angular;
	   else if(vangle_key<0)  vlinear_z.data=-v_angular; 
	   else vlinear_z.data=0;
	   //处理数据
	   if(Joy->buttons[1]==1) //按下B键时，切换为麦轮车，可左右平移
	     flag_mec=1;
	   if(Joy->buttons[0]==1) //按下A键时，恢复正常转向模式
	     flag_mec=0;
	   if(flag_mec) 
	   {
	     v.linear.y=0.2*acce_z*vlinear_z.data;
	     v.linear.x = vlinear_x.data*acce_x;
	     v.angular.z=0;
	   }
	   else
	   {
	   v.linear.x = vlinear_x.data*acce_x;
	   v.angular.z = dir*vlinear_z.data*acce_z;
	   }
	   //打印输出
	   //ROS_INFO("linear:%.3lf angular:%.3lf",vlinear_x.data,v.angular.z);
	   pub->publish(v);
	}


    //机器人的初始速度
    double v_linear,v_angular;
    //手柄键值
    int axis_angular,axis_linear; 
    int dir,flag_mec;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    printf("joy_control\n");
    rclcpp::spin(std::make_shared<wheeltec_joy>());
    return 0;
}
