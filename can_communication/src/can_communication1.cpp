#include<stdint.h>
#include<stdio.h>
#include <ros/ros.h>											// 包含ROS的头文件
#include <ros/time.h>
#include <math.h>
#include <array>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include"can_communication/motor_control.h"
    Motor_Control motor_Ctr;

#define		PI			3.1415926535897932f
#define   R       0.25
#define   r_to_m_Switch  9000.0
ros::Time current_time, last_time;

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9}};
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9}};


struct robotSpeed1{
  INT16 Motor_ID=0x202;  
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed;
 
} left_Wheel;


struct robotSpeed2{
  INT16 Motor_ID=0x201;  
  BYTE Control_Word[2]={0x0f,0x00} ;
  BYTE Work_Mode=0x03;
  int Speed;
 
} right_Wheel;

struct robotSpeed3{
  INT16 Motor_ID=0x203;  
  BYTE Control_Word1[2]={0x1f,0x00} ;
  BYTE Control_Word2[3]={0x2f,0x00} ;
  BYTE Work_Mode=0x01;
  unsigned int absolute_position;
  int Speed;
 
} up_down_motor;





double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

union odometry													//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right, temprature, odom_yaw, odom_pitch, odom_roll;

void readData()
{
  motor_Ctr.Motor_Feedback();
  ros::Time curr_time;
  vx= (motor_Ctr.left_realtime_Speed+motor_Ctr.right_realtime_Speed)*PI/(2*r_to_m_Switch);
  vth=((motor_Ctr.right_realtime_Speed-motor_Ctr.left_realtime_Speed)*PI/r_to_m_Switch)/(2*R);
  curr_time = ros::Time::now();

  double dt = (curr_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
  last_time = curr_time;
  ROS_INFO("vx:%f,vth:%f",vx,vth) ;
  ROS_INFO("x:%f,y:%f",x,y) ;
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
		left_Wheel.Speed =-RobotV;
		right_Wheel.Speed =RobotV;
	}
	else
	{
		right_Wheel.Speed =(YawRate * (r + R)*r_to_m_Switch)/PI;
		left_Wheel.Speed = -(YawRate * (r - R)*r_to_m_Switch)/PI;
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
		       ros::Rate loop_rate(20);
	      	 ros::NodeHandle nh;
           ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 50); 
           tf::TransformBroadcaster odom_broadcaster;
          
           ros::Subscriber sub = nh.subscribe("cmd_vel", 50, &writeSpeed);
         
           geometry_msgs::TransformStamped odom_trans;
           motor_Ctr.Motor_PDO_Open();
           motor_Ctr.Motor_Speed_Control(left_Wheel.Motor_ID, 0, left_Wheel.Work_Mode,left_Wheel.Control_Word);
        	 motor_Ctr.Motor_Speed_Control(right_Wheel.Motor_ID,0, right_Wheel.Work_Mode,right_Wheel.Control_Word);
           ROS_INFO("ROS Node initialized successful.");
           nav_msgs::Odometry msgl;
           while(ros::ok())
           {
                
                readData();
                current_time = ros::Time::now();
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);        /*角度转换成四元数坐标*/    
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                       


               //send the transform
                odom_broadcaster.sendTransform(odom_trans);


                msgl.header.stamp = current_time;
                msgl.header.frame_id = "odom";
                /*设置位置*/
                msgl.pose.pose.position.x = x;
                msgl.pose.pose.position.y = y;
                msgl.pose.pose.position.z = 0.0;
                msgl.pose.pose.orientation = odom_quat;
                msgl.pose.covariance=odom_pose_covariance;
                /*设置方向*/
                msgl.child_frame_id = "base_link";
                msgl.twist.twist.linear.x = vx;
                msgl.twist.twist.linear.y = vy;
                msgl.twist.twist.angular.z = vth;
                msgl.twist.covariance=odom_twist_covariance;
   
                pub.publish(msgl);   
                ros::spinOnce();
                loop_rate.sleep();
           }
          
         
           return 0;
  }
