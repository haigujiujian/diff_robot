    #include<stdint.h>
    #include<stdio.h>
    #include <ros/ros.h>											// 包含ROS的头文件
    #include <ros/time.h>
    #include <math.h>
    #include <geometry_msgs/Twist.h>
    #include"can_communication/motor_control.h"
    Motor_Control motor_Ctr;

#define		PI			3.1415926535897932f
#define   R       0.25
#define   r_to_m_Switch  9000
ros::Time current_time, last_time;
struct robotSpeed1{
  INT16 Motor_ID=0x201;  
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed;
 
} left_Wheel;


struct robotSpeed2{
  INT16 Motor_ID=0x202;  
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed;
 
} right_Wheel;

union odometry													//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right, temprature, odom_yaw, odom_pitch, odom_roll;

void readData()
{
  motor_Ctr.Motor_Feedback();
  ros::Time curr_time;

}

void writeSpeed(const geometry_msgs::Twist& msg)
{
	double RobotV = (msg.linear.x*r_to_m_Switch)/PI;
	double YawRate = msg.angular.z;
	double r = msg.linear.x / YawRate;
	if(RobotV == 0)
	{
		left_Wheel.Speed = (YawRate * R*r_to_m_Switch)/PI;
		right_Wheel.Speed = (YawRate * R*r_to_m_Switch)/PI;
	} else if(YawRate == 0)
	{
		left_Wheel.Speed = RobotV;
		right_Wheel.Speed =-RobotV;
	}
	else
	{
		right_Wheel.Speed = -(YawRate * (r - R)*r_to_m_Switch)/PI;
		left_Wheel.Speed =(YawRate * (r + R)*r_to_m_Switch)/PI;
	}
  ROS_INFO("left_wheel_Speed:%d",left_Wheel.Speed);
  ROS_INFO("right_wheel_Speed:%d",right_Wheel.Speed);
  motor_Ctr.Motor_Speed_Control(left_Wheel.Motor_ID, left_Wheel.Speed, left_Wheel.Work_Mode,left_Wheel.Control_Word);
	motor_Ctr.Motor_Speed_Control(right_Wheel.Motor_ID, right_Wheel.Speed, right_Wheel.Work_Mode,right_Wheel.Control_Word);
}

   int main(int argc, char** argv)
  {
       
     
           ros::init(argc, argv, "can_communication1");									//初始化节点
	         ros::Time::init();
		       current_time = ros::Time::now();
		       last_time = ros::Time::now();
		       ros::Rate loop_rate(50);
	      	 ros::NodeHandle nh;
           ros::Subscriber sub = nh.subscribe("cmd_vel", 50, &writeSpeed);
           ROS_INFO("ROS Node initialized successful.");
           motor_Ctr.Motor_PDO_Open();
		       ros::spin();
          
         //motor_Ctr.Motor_Feedback();
           return 0;
  }
