#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "stepper_arm/arm_control.h"
#include "stepper_arm/keyboard_control.h"

using namespace std;

ArmKeyboardControlNode::ArmKeyboardControlNode() : Node("arm_keyboard_control_node") {
    timer = this->create_wall_timer(std::chrono::microseconds(1), std::bind(&ArmKeyboardControlNode::timer_callback, this));
    joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Arm_2_JointStates", 10);
    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->get_parameter_or<double>("Angle_B_bias", Angle_B_bias, 1.571);
    this->get_parameter_or<double>("Angle_C_bias", Angle_C_bias, 0.384);
    this->get_parameter_or<double>("arm_base_height", arm_base_height, 0);
    this->get_parameter_or<bool>("cmd_vel_enable", cmd_vel_enable, true);

    key_scan_thread = std::thread(&ArmKeyboardControlNode::Threading_key_scan, this);
}

void ArmKeyboardControlNode::timer_callback() {
    sensor_msgs::msg::JointState joint_states;
    joint_states.position.push_back(Joint_A);
    joint_states.position.push_back(Joint_B-Angle_B_bias);
    joint_states.position.push_back(Joint_C-Angle_C_bias);
    joint_states.position.push_back(Joint_End);
    joint_states.position.push_back(Joint_Hand_left1);
    joint_states.position.push_back(Joint_Hand_left2);
    joint_states.position.push_back(Joint_Hand_right1);
    joint_states.position.push_back(Joint_Hand_right2);
    joint_states.position.push_back(Joint_Grasper);
    joint_state_publisher->publish(joint_states);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x =Car_foward_velocity;
    cmd_vel.linear.y =Car_lateral_velocity;

    cmd_vel.linear.z =0;
    cmd_vel.angular.x=0;
    cmd_vel.angular.y=0;
    cmd_vel.angular.z=Car_turn_velocity;
    if (cmd_vel_enable) cmd_vel_publisher->publish(cmd_vel);
    std::count <<key_wait_count<<std::endl;
    key_wait_count++;
    if (key_wait_count>30) {
        Car_foward_velocity=0,
        Car_lateral_velocity=0,
        Car_turn_velocity=0;
    }
}

/**************************************************************************
Function: Gets the key value of the keyboard pressed
Input   : none
Output  : key value
函数功能： 获取键盘按下键值
入口参数： 无
返回  值： 键值
**************************************************************************/
int ArmKeyboardControlNode::scan_keyboard()
{
    int in;

    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);      
    in = getchar(); 
    tcsetattr(0,TCSANOW,&stored_settings);

    return in;
}
/**************************************************************************
Function: Multiple threads get key values
Input   : none
Output  : none
函数功能：多线程获取键值
入口参数：无
返回  值：无
**************************************************************************/
void ArmKeyboardControlNode::Threading_key_scan()
{
    while(1)
    {
        char Key; //键值 
        static double step = 0.005; //键盘控制默认步长 
        
        //打印键盘控制提示信息
        cout<<endl<<"---------------------------"<<endl
            <<"Control your stepperArm."<<endl         
            <<"q/a : Arm_A_Joint increase/decrease 1 step(rad)"<<endl
            <<"w/s : Arm_B_Joint increase/decrease 1 step(rad)"<<endl
            <<"e/d : Arm_C_Joint increase/decrease 1 step(rad)"<<endl
            <<"r/f : Arm_end_point foward/back 1 step(mm)"<<endl
            <<"t/g : Arm_end_point up/down 1 step(mm)"<<endl
            <<"y/h : Arm_hand tighten/loosen 1 step(rad)"<<endl<<endl

            <<"Control your ROS_Car."<<endl
            <<"Direction instrument:"<<endl
            <<"u    i    o"<<endl
            <<"j    k    l"<<endl
            <<"m    ,    ."<<endl
            <<"k : stop ROS_Car"<<endl
            <<"b : switch to OmniMode/CommonMode"<<endl
            <<"speed = step*10 = "<<step*10<<endl<<endl

            <<"z/x : increase/decrease step 0.001"<<endl
            <<"step: "<<step<<endl;
        cout<<"Esc : to quit"<<endl<<endl;

        //打印机械臂各关节状态和机械臂杆C终点位置
        cout<<"Joint_A: "<<Joint_A<<endl; 
        cout<<"Joint_B: "<<Joint_B<<endl; 
        cout<<"Joint_C: "<<Joint_C<<endl; 
        cout<<"Joint_Hand: "<<Joint_Hand_right1<<endl; 
        cout<<"PointX: "<<ArmC_End_Position_X<<endl; 
        cout<<"PointY: "<<ArmC_End_Position_Y<<endl<<endl; 

        cout<<"Key: ";
        Key = scan_keyboard(); //读取键盘键值
        key_wait_count=0;
        cout<<endl<<endl;
        /*1.控制机械臂关节*/
        if(Key=='q' || Key=='Q') 
        {
            cout<<"joint_A state increase "<< step <<" rad"<<endl;
            Joint_A = Joint_A+step;
            if(Joint_A>AngleA_Max)Joint_A=AngleA_Max;
        }
        else if(Key=='a' || Key=='A') 
        {
            cout<<"joint_A state decrease "<< step <<" rad"<<endl;
            Joint_A = Joint_A-step;
            if(Joint_A<AngleA_Min)Joint_A=AngleA_Min;
        }
        else if(Key=='w' || Key=='W')
        {
            cout<<"joint_B state increase "<< step <<" rad"<<endl;
            Joint_B = Joint_B+step;
            //控制关节运动后，要正解计算运动后的机械臂杆C终点位置
            arm_2_positive_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(Key=='s' || Key=='S')
        {
            cout<<"joint_B state decrease "<< step <<" rad"<<endl;
            Joint_B = Joint_B-step;
            //控制关节运动后，要正解计算运动后的机械臂杆C终点位置
            arm_2_positive_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(Key=='e' || Key=='E')
        {
            cout<<"joint_C state increase "<< step <<" rad"<<endl;
            Joint_C = Joint_C+step;
            //控制关节运动后，要正解计算运动后的机械臂杆C终点位置
            arm_2_positive_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        }
        else if(Key=='d' || Key=='D')
        {
            cout<<"joint_C state decrease "<< step <<" rad"<<endl;
            Joint_C = Joint_C-step;   
            //控制关节运动后，要正解计算运动后的机械臂杆C终点位置
            arm_2_positive_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);      
        }

        /*2.控制机械臂向上下前后方向移动*/
        else if(Key=='r' || Key=='R')
        {
            //控制机械臂杆C终点运动，内部自动执行逆解计算关节状态
            Position_move(foward, step);          
        }
        else if(Key=='f' || Key=='F')
        {
            //控制机械臂杆C终点运动，内部自动执行逆解计算关节状态
            Position_move(back, step);          
        }
        else if(Key=='t' || Key=='T')
        {
            //控制机械臂杆C终点运动，内部自动执行逆解计算关节状态
            Position_move(up, step);          
        }
        else if(Key=='g' || Key=='G')
        {
            //控制机械臂杆C终点运动，内部自动执行逆解计算关节状态
            Position_move(down, step);          
        }

        /*3.控制机械爪夹紧/松开*/
        else if(Key=='y' || Key=='Y')
        {
            //控制机械爪夹紧
            cout<<"joint_Hand state increase "<< 2*step <<" rad"<<endl;   
            Joint_Hand_left1 = Joint_Hand_left1+step*10;
            Joint_Hand_right1 = Joint_Hand_right1-step*10;  
            if(Joint_Hand_right1<AngleD_Min)Joint_Hand_right1=AngleD_Min;
            if(Joint_Hand_left1>AngleD_Max) Joint_Hand_left1=AngleD_Max;  
        }
        else if(Key=='h' || Key=='H')
        {
            //控制机械爪松开
            cout<<"joint_Hand state decrease "<< 2*step <<" rad"<<endl;   
            Joint_Hand_left1 = Joint_Hand_left1-step*10;
            Joint_Hand_right1 = Joint_Hand_right1+step*10; 
            if(Joint_Hand_left1<AngleD_Min) Joint_Hand_left1=AngleD_Min;
            if(Joint_Hand_right1>AngleD_Max)Joint_Hand_right1=AngleD_Max;       
        }

        /*4.控制小车运动*/
        //注意这里以机械臂默认朝向为前进正方向
        else if(Key=='i' || Key=='I')
        {
            cout<<"ROS_Car move foward, speed: "<< 10*step <<" m/s"<<endl;   
            Car_foward_velocity =  10*step;   
        }
        else if(Key==',')
        {
            cout<<"ROS_Car move back, speed: "<< 10*step <<" m/s"<<endl;   
            Car_foward_velocity =  -10*step;   
        }
        else if(Key=='j' || Key=='J')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move left, speed: "<< 10*step <<" m/s"<<endl;   
                Car_lateral_velocity = 10*step; 
            }
            else
            {
                cout<<"ROS_Car turn left, speed: "<< 10*step <<" rad/s"<<endl;   
                Car_turn_velocity = 10*step; 
            }
              
        }
        else if(Key=='l' || Key=='L')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move right, speed: "<< 10*step <<" m/s"<<endl;   
                Car_lateral_velocity = -10*step; 
            }
            else
            {
                cout<<"ROS_Car turn right, speed: "<< 10*step <<" rad/s"<<endl;   
                Car_turn_velocity = -10*step; 
            }  
        }
        else if(Key=='u' || Key=='U')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move foward and left, speed: "<< 10*step <<" m/s"<<endl;   
                Car_foward_velocity  =  10*step; 
                Car_lateral_velocity =  10*step; 
            }
            else
            {
                cout<<"ROS_Car move foward and turn left, speed: "<< 10*step <<" m/s | rad/s"<<endl;   
                Car_foward_velocity =  10*step; 
                Car_turn_velocity   =  10*step; 
            }  
        }
        else if(Key=='o' || Key=='O')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move foward and right, speed: "<< 10*step <<" m/s"<<endl;   
                Car_foward_velocity  =  10*step; 
                Car_lateral_velocity = -10*step; 
            }
            else
            {
                cout<<"ROS_Car move foward and turn right, speed: "<< 10*step <<" m/s | rad/s"<<endl;   
                Car_foward_velocity =  10*step; 
                Car_turn_velocity   = -10*step; 
            }  
        }
        else if(Key=='m' || Key=='M')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move back and left, speed: "<< 10*step <<" m/s"<<endl;   
                Car_foward_velocity  = -10*step; 
                Car_lateral_velocity =  10*step; 
            }
            else
            {
                cout<<"ROS_Car move back and turn left, speed: "<< 10*step <<" m/s | rad/s"<<endl;   
                Car_foward_velocity = -10*step; 
                Car_turn_velocity   = -10*step; 
            }  
        }
        else if(Key=='.')
        {
            if(Omni==true)
            {
                cout<<"ROS_Car move back and right, speed: "<< 10*step <<" m/s"<<endl;   
                Car_foward_velocity  = -10*step; 
                Car_lateral_velocity = -10*step; 
            }
            else
            {
                cout<<"ROS_Car move back and turn right, speed: "<< 10*step <<" m/s | rad/s"<<endl;   
                Car_foward_velocity = -10*step; 
                Car_turn_velocity   =  10*step; 
            }  
        }
        else if(Key=='k' || Key=='K')
        {
            Car_foward_velocity=0, Car_lateral_velocity=0, Car_turn_velocity=0;
        }
        else if(Key=='b' || Key=='B')
        {
            Omni=!Omni;
            if(Omni)cout<<"switched to OmniMode"<<endl;  
            else    cout<<"switched to CommonMode"<<endl;  
        }

        /*5.调整机械臂、小车运动速度*/
        else if(Key=='z' || Key=='Z')
        {
            //增加键盘控制步长
            cout<<"step increase "<< step <<" unit"<<endl;   
            step=step+0.001;
        }
        else if(Key=='x' || Key=='Z')
        {
            //降低键盘控制步长
            cout<<"step decrease "<< step <<" unit"<<endl; 
            step=step-0.001;
        }

        /*5.终止程序*/
        else if(Key==27) //Esc键终止程序
        {
            cout<<"Programe 'arm_2_keyboard_control' terminate. "<<endl; 
            Programe_terminate=1;
            break;
        }
        else
        {
            Car_foward_velocity=0, Car_lateral_velocity=0, Car_turn_velocity=0;
        }
        
    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmKeyboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}