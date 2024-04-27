
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h>      
#include <rcl/types.h>
#include <sys/stat.h>

#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>     // CHANGE
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>


using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84	
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	

extern sensor_msgs::msg::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;
}RECEIVE_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot : public rclcpp::Node

{
	public:
		turn_on_robot();
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
                void update_tfupdate_tf(geometry_msgs::msg::TransformStamped::SharedPtr odom_tf);
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象 
		//explicit turn_on_robot(
		 // const std::string & name = "wheeltec_robot");

	private:

		//rclcpp::Node::SharedPtr nl;           //Create a ROS node handle //创建ROS节点句柄
		rclcpp::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)
		//rclcpp::Subscriber Cmd_Vel_Sub; //Initialize the topic subscriber //初始化话题订阅者
		//Ackerman speed topic subscribe callback function, type Ackerman trolleys use this function declaration
		//阿克曼速度话题订阅回调函数，类型阿克曼小车使用此函数声明
		//The speed topic subscribes to the callback function, which is used by other cars to declare
		//速度话题订阅回调函数，其它小车使用此函数声明
		//void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);       

  		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
  		rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub;

                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;         // CHANGE
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;         // CHANGE
                rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;         // CHANGE

                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher;         // CHANGE

                rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr robotpose_publisher;         // CHANGE
                rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr robotvel_publisher;         // CHANGE
                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tf_pub_;

                std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;
                rclcpp::TimerBase::SharedPtr test_timer;

                rclcpp::TimerBase::SharedPtr odom_timer;
                rclcpp::TimerBase::SharedPtr imu_timer;
                rclcpp::TimerBase::SharedPtr voltage_timer;

                rclcpp::TimerBase::SharedPtr robotpose_timer;
                rclcpp::TimerBase::SharedPtr robotvel_timer;

                std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

                void declare_parameters();
                void get_parameters();


                void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
                void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl);
                //void Akm_Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
		void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
		void Publish_Voltage();   //Pub the power supply voltage topic //发布电源电压话题
		auto createQuaternionMsgFromYaw(double yaw);
 
        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();   
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id, akm_cmd_vel, test; //Define the related variables //定义相关变量
        std::string cmd_vel;
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;       //Power supply voltage //电源电压
    size_t count_;
};


#endif
