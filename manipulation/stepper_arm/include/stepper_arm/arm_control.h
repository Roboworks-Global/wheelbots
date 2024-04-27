#ifndef __ARM_2_CONTROL_H_
#define __ARM_2_CONTROL_H_

#include <string.h> 

#define PI          3.1415926536f
#define RadiusB     0.1400f //杆B长度
#define RadiusC     0.1600f //杆C长度
#define AngleB_Max  1.5708f //关节B角度最大值(相对X轴)，顺时针为负
#define AngleB_Min -0.1700f //关节B角度最小值(相对X轴)，逆时针为正
#define AngleC_Max  0.7273f //关节C角度最大值(相对X轴)，顺时针为负
#define AngleC_Min -1.1790f //关节C角度最小值(相对X轴)，逆时针为正

#define AngleA_Max  2.2500f //底座A角度最大值(相对初始位置)，顺时针为负
#define AngleA_Min -2.2500f //底座A角度最小值(相对初始位置)，逆时针为正
#define AngleD_Max  1.0000f //机械爪D角度最大值(相对初始位置)，夹紧为负
#define AngleD_Min -1.0000f //机械爪D角度最小值(相对初始位置)，张开为正

//枚举，关节C终点运动方向
enum direction{foward, back, up, down, foward_up, foward_down, back_up, back_down};
extern double arm_base_height;
extern double Angle_B_bias, Angle_C_bias; //关节B默认角度(相对X轴)、关节C默认角度(相对关节B)
extern double Joint_A, Joint_B, Joint_C, Joint_End, Joint_Grasper;
extern double Joint_Hand_left1, Joint_Hand_left2, Joint_Hand_right1, Joint_Hand_right2;
extern double ArmC_End_Position_X, ArmC_End_Position_Y;

void arm_2_inverse_solution(double &x, double &y, double &Joint_B_Radian, double &Joint_C_Radian);
void arm_2_positive_solution(double &x, double &y, double &Joint_B_Radian, double &Joint_C_Radian);
int arm_range_judge(double x, double y);
int Position_move(int direction, float step);

//cout related
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
//cout related
#endif