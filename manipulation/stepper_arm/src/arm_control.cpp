#include <math.h>
#include <iostream>

#include "stepper_arm/arm_control.h"

using namespace std;

//机械臂底座到地面的距离
double arm_base_height=0.15; 
//关节B默认角度(相对X轴，逆时针为正)、关节C默认角度(相对关节B，逆时针为正)，用于控制机械臂
double Angle_B_bias = 1.570796, Angle_C_bias = 0.391796; 

//---机械臂初始关节角度，Joint_B代表杆B与X正轴的夹角，Joint_C代表杆C与杆B的夹角---//
double Joint_A = 0, Joint_B = Angle_B_bias, Joint_C = Angle_C_bias, Joint_End = 0, Joint_Grasper = 0;
double Joint_Hand_left1 = 0, Joint_Hand_left2 = 0, Joint_Hand_right1 = 0, Joint_Hand_right2 = 0;
//---机械臂关节C终点的初始位置---//
double ArmC_End_Position_X=0.062, ArmC_End_Position_Y=-0.008;

/**************************************************************************
Function: The inverse solution of the two-degree-of-freedom manipulator is used to calculate the Angle of the target joint from the position of the target point
Input   : Target position, target joint Angle
Output  : none
函数功能：二自由度机械臂逆解，由目标点位置，求目标关节角度
入口参数：目标位置，目标关节角度
返回  值：无
**************************************************************************/
void arm_2_inverse_solution(double &x, double &y, double &Joint_B_Radian, double &Joint_C_Radian)
{
    //目标位置与坐标系原点的距离
	auto Radius_xy=pow(x*x+y*y, 0.5);
	//cout<<"Radius_xy: "<<Radius_xy<<endl;

    //由余弦定理求由求关节B角度1
	auto JointB_Angle1=acos((RadiusB*RadiusB+Radius_xy*Radius_xy-RadiusC*RadiusC)/(2*RadiusB*Radius_xy));
	//逆正切求由求关节B角度2
    auto JointB_Angle2=atan(y/x);
    //cout<<"JointB_Angle1: "<<JointB_Angle1<<endl;
    //cout<<"JointB_Angle2: "<<JointB_Angle2<<endl;

    //关节B角度1、2相加求出关节B目标角度(关节B与横轴的夹角)
    Joint_B_Radian=JointB_Angle1+JointB_Angle2;
    //由余弦定理求由求关节C目标角度(关节C与关节B的夹角)
    Joint_C_Radian=acos((RadiusB*RadiusB+RadiusC*RadiusC-Radius_xy*Radius_xy)/(2*RadiusB*RadiusC));
   
    //cout<<"Joint_B_Radian: "<<Joint_B_Radian<<endl;
    //cout<<"Joint_C_Radian: "<<Joint_C_Radian<<endl;  
}
/**************************************************************************
Function: The forward solution of the two-degree-of-freedom manipulator, the position of the target point is obtained from the Angle of the target joint
Input   : Target position, target joint Angle
Output  : none
函数功能：二自由度机械臂正解，由目标关节角度，求目标点位置
入口参数：目标位置，目标关节角度
返回  值：无
**************************************************************************/
void arm_2_positive_solution(double &x, double &y, double &Joint_B_Radian, double &Joint_C_Radian)
{
    //逆解中间变量
    double PointB_x, PointB_y, PointC_x, PointC_y;
    static double last_Joint_B_Radian, last_Joint_C_Radian;

    if(Joint_B_Radian<AngleB_Min)
    {
        cout<<RED<<"ERROR: Joint_B_Radian<"<<AngleB_Min<<RESET<<endl;
        Joint_B_Radian=AngleB_Min;
    }
    if(Joint_B_Radian>AngleB_Max)
    {
        cout<<RED<<"ERROR: Joint_B_Radian>"<<AngleB_Max<<RESET<<endl;
        Joint_B_Radian=AngleB_Max;
    }
    if(Joint_C_Radian<(AngleC_Min+PI/2)) 
    {
        //注意Joint_C_Radian代表的是机械臂杆B、C之间的夹角，而AngleC_Min代表的是杆C与横X正轴的夹角(逆时针为正)
        cout<<RED<<"ERROR: Joint_C_Radian<"<<(AngleC_Min+PI/2)<<RESET<<endl;
        Joint_C_Radian=(AngleC_Min+PI/2);
    }
    if(Joint_C_Radian>(AngleC_Max+PI/2))
    {
        //注意Joint_C_Radian代表的是机械臂杆B、C之间的夹角，而AngleC_Min代表的是杆C与横X正轴的夹角(逆时针为正)
        cout<<RED<<"ERROR: Joint_C_Radian>"<<(AngleC_Max+PI/2)<<RESET<<endl;
        Joint_C_Radian=(AngleC_Max+PI/2);
    }
    if(Joint_B_Radian+Joint_C_Radian<PI/2)
    {
        cout<<RED<<"ERROR: Joint_B_Radian+Joint_C_Radian<PI/2"<<RESET<<endl;
        Joint_B_Radian=last_Joint_B_Radian;
        Joint_C_Radian=last_Joint_C_Radian;
    }

    PointB_x=RadiusB*cos(Joint_B_Radian);
    PointB_y=RadiusB*sin(Joint_B_Radian);
    PointC_x=PointB_x+RadiusC*cos(Joint_C_Radian-3.14159+Joint_B_Radian);
    PointC_y=PointB_y+RadiusC*sin(Joint_C_Radian-3.14159+Joint_B_Radian);

    x=PointC_x;
    y=PointC_y;

    last_Joint_B_Radian=Joint_B_Radian;
    last_Joint_C_Radian=Joint_C_Radian;
}

/**************************************************************************
Function: Inverse solution of two degrees of freedom manipulator
Input   : Target position, target joint Angle
Output  : none
函数功能：判断目标位置是否在机械臂运动范围内
入口参数：目标位置
返回  值：超出运动范围：1，在运动范围内：0
**************************************************************************/
int arm_range_judge(double x, double y)
{
	//中间变量
	int Judge1=0, Judge2=0, Judge3=0, Judge4=0, Judge5=0, Judge6=0;
	float Radius1, Radius2, Radius3, Radius4;
    float PointD_x, PointD_y, RadiusD;
    float PointE_x, PointE_y, RadiusE;
    float Center1_x, Center1_y, Center2_x, Center2_y, Center3_x, Center3_y, Center4_x, Center4_y;

    //---运动范围由4个圆弧+1个最低高度组成

    //圆弧1的圆心点
    Center1_x=0;
    Center1_y=RadiusB;
    //圆弧1的半径即为杆C长度

    //圆弧2的圆心点
    Center2_x=0;
    Center2_y=0;
    //圆弧2的起点位置
    PointD_x=0+RadiusC*cos(AngleC_Min);
    PointD_y=RadiusB+RadiusC*sin(AngleC_Min);
    //由圆弧2的起点位置，求圆弧2的半径
    RadiusD=pow(PointD_x*PointD_x+PointD_y*PointD_y, 0.5);

    //圆弧3的圆心点
    Center3_x=RadiusB*cos(AngleB_Min);
    Center3_y=RadiusB*sin(AngleB_Min);
    //圆弧3的半径即为杆C长度

    //圆弧4的圆心点
    Center4_x=0;
    Center4_y=0;
    //圆弧4的起点位置
    PointE_x=0+RadiusC*cos(AngleC_Max);
    PointE_y=RadiusB+RadiusC*sin(AngleC_Max);
    //由圆弧4的起点位置，求圆弧4的半径
    RadiusE=pow(PointE_x*PointE_x+PointE_y*PointE_y, 0.5);

	/*cout<<"RadiusB: "<<RadiusB<<endl;
	cout<<"RadiusC: "<<RadiusC<<endl;
	cout<<"RadiusD: "<<RadiusD<<endl;
	cout<<"RadiusE: "<<RadiusE<<endl;*/

    //求目标点与4个圆弧中心的距离
    Radius1 = pow((x-Center1_x)*(x-Center1_x)+(y-Center1_y)*(y-Center1_y),0.5);
	Radius2 = pow((x-Center2_x)*(x-Center2_x)+(y-Center2_y)*(y-Center2_y),0.5);
	Radius3 = pow((x-Center3_x)*(x-Center3_x)+(y-Center3_y)*(y-Center3_y),0.5);
	Radius4 = pow((x-Center4_x)*(x-Center4_x)+(y-Center4_y)*(y-Center4_y),0.5);

	/*cout<<"Radius1: "<<Radius1<<endl;
	cout<<"Radius2: "<<Radius2<<endl;
	cout<<"Radius3: "<<Radius3<<endl;
	cout<<"Radius4: "<<Radius4<<endl;*/

    //如果目标点与圆弧1中心的距离 小于 圆弧1半径(杆C长度)，则超出机械臂运动范围
	if(Radius1<RadiusC) Judge1=1;
	else                Judge1=0;
	//如果目标点与圆弧2中心的距离 小于 圆弧2半径，则超出机械臂运动范围
	if(Radius2<RadiusD) Judge2=1;
	else                Judge2=0;
	//如果目标点与圆弧3中心的距离 大于 圆弧3半径(杆C长度) ，同时目标点y值小于0(因为圆弧3位于y轴负半轴)，则超出机械臂运动范围
	if(Radius3>RadiusC && y<0) Judge3=1;
	else                        Judge3=0;
	//如果目标点与圆弧4中心的距离 大于 圆弧4半径，则超出机械臂运动范围
	if(Radius4>RadiusE) Judge4=1;
	else                Judge4=0;
	//如果目标点y值小于关节B与地面的距离，则超出机械臂运动范围
	if(y<-(arm_base_height)) Judge5=1; //计算关节B起点到地面的距离，然后判断目标点是否超出运动范围
	else                     Judge5=0;
   
    //由于机械结构原因，机械臂的C杆与水平面(X轴)的夹角不能小于-90°而产生的运动范围
    if(Judge1==0 && Judge2==0 && Judge3==0 && Judge4==0 && Judge5==0 )
    {  
        double angle_B, angle_C;
        arm_2_inverse_solution(x, y, angle_B, angle_C);
        if((angle_B+angle_C)<PI/2) Judge6=1;
        else                       Judge6=0;
    }
    /*cout<<"arm_base_height: "<<arm_base_height<<endl;
	cout<<"Judge1: "<<Judge1<<endl;
	cout<<"Judge2: "<<Judge2<<endl;
	cout<<"Judge3: "<<Judge3<<endl;
	cout<<"Judge4: "<<Judge4<<endl;
	cout<<"Judge5: "<<Judge5<<endl;
    cout<<"Judge6: "<<Judge6<<endl;*/

    //任意判断不成立，则超出机械臂运动范围
    int error=0;
    if(Judge1)error=error+1;
    if(Judge2)error=error+2;
    if(Judge3)error=error+4;
    if(Judge4)error=error+8;
    if(Judge5)error=error+16;
    if(Judge6)error=error+32;
	return error;
}

/**************************************************************************
Function: According to the displacement of the target direction, the target position is calculated, and the joint Angle is obtained by inverse solution
Input   : Robot arm exchange object, target displacement direction, target step length
Output  : none
函数功能：根据目标方向位移，计算目标位置，逆解求出关节角度
入口参数：方向，步长
返回  值：无
**************************************************************************/
int Position_move(int direction, float step)
{
    int error=0;
    //根据目标
    if(direction==0) //前进方向
    {
        cout<<"Planning foward "<< step <<"m"<<endl;
        ArmC_End_Position_X = ArmC_End_Position_X+step;
    }
    if(direction==1) //后退方向
    {
        cout<<"Planning back "<< step <<"m"<<endl;
        ArmC_End_Position_X = ArmC_End_Position_X-step;
    }
    if(direction==2) //上升方向
    {
        cout<<"Planning up "<< step <<"m"<<endl;
        ArmC_End_Position_Y = ArmC_End_Position_Y+step;
    }
    if(direction==3) //下降方向
    {
        cout<<"Planning down "<< step <<"m"<<endl;
        ArmC_End_Position_Y = ArmC_End_Position_Y-step;
    }
 
    cout<<"Target_X: "<<ArmC_End_Position_X<<endl;
    cout<<"Target_Y: "<<ArmC_End_Position_Y<<endl;

    //判断关节C终点目标位置是否超出机械臂运动范围
    if(arm_range_judge(ArmC_End_Position_X, ArmC_End_Position_Y)==0)
    {
        //打印
        //cout<<"AngleB: "<<Joint_B<<"(rad)/"<<Joint_B*57.3<<"(°)"<<endl;
        //cout<<"AngleC: "<<Joint_C<<"(rad)/"<<Joint_C*57.3<<"(°)"<<endl;

        //关节C终点目标位置在机械臂运动范围内，进行逆解，求关节B目标角度(相对X轴)、关节C目标角度(相对关节B)
        cout<<GREEN<<"Target position does fit in range of motion."<<RESET<<endl<<endl;
        arm_2_inverse_solution(ArmC_End_Position_X, ArmC_End_Position_Y, Joint_B, Joint_C);
        error = 0;
    }
    else 
    {
        error = arm_range_judge(ArmC_End_Position_X, ArmC_End_Position_Y);
        //关节C终点目标位置超出机械臂运动范围
        cout<<RED<<"ERROR: Target position out of range of motion !"<<RESET<<endl<<endl;
        //重置目标点位置回到上一个位置
        if(direction==0) //前进方向
        {
            ArmC_End_Position_X = ArmC_End_Position_X-step;
        }
        if(direction==1) //后退方向
        {
            ArmC_End_Position_X = ArmC_End_Position_X+step;
        }
        if(direction==2) //上升方向
        {
            ArmC_End_Position_Y = ArmC_End_Position_Y-step;
        }
        if(direction==3) //下降方向
        {
            ArmC_End_Position_Y = ArmC_End_Position_Y+step;
        }
    }
    return error;
}