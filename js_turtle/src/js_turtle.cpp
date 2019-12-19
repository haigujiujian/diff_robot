#include<ros/ros.h>  
#include<geometry_msgs/Twist.h>  
#include <sensor_msgs/Joy.h>  
#include<iostream>  
using namespace std;  
 
class Teleop  
{  
 public:  
    Teleop();  
    double v1 = 0.2,w1 =0.2 ;
    double vmax=0.5,vmin=0.1,ang_max=0.6,ang_min=0.1;
  
//private:  
    /* data */  
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);  
    ros::NodeHandle n; //实例化节点  
    ros::Subscriber sub ;  
    ros::Publisher pub ;  
    double vlinear,vangular;//我们控制乌龟的速度，是通过这两个变量调整  
    int axis_ang,axis_lin,ton0,ton1, ton7,ton6,ton2,ton3;  //axes[]的键  
 
};  
  
Teleop::Teleop()  
{     

    n.param<int>("axis_linear",axis_lin,1); //默认axes[1]接收速度   
    n.param<int>("axis_angular",axis_ang,0);//默认axes[2]接收角度
    n.param<double>("vel_linear",vlinear,v1);//默认线速度1 m/s
    n.param<double>("vel_angular",vangular,w1);//默认角速度1 单位rad/s
    n.param<int>("button",ton6,6);
    n.param<int>("button",ton7,7);
    n.param<int>("button",ton0,0);
    n.param<int>("button",ton1,1);
    n.param<int>("button",ton2,2);
    n.param<int>("button",ton3,3);
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);//将速度发给乌龟
    sub = n.subscribe<sensor_msgs::Joy>("joy",10,&Teleop::callback,this); //订阅游戏手柄发来的数据
    //ROS_INFO("%f,%f",vlinear,vangular);
}
void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)  
{   
    
    geometry_msgs::Twist v;
   
    if(Joy->buttons[ton6] | Joy->buttons[ton7] )
   {
     if(Joy->buttons[ton0])
     {
         if(vlinear<vmax)
         {
             vlinear=vlinear+0.01;
         }
     }
     else if(Joy->buttons[ton2])
     {
         if(vlinear>vmin)
         {
             vlinear=vlinear-0.01;
         }
     }
     else if(Joy->buttons[3])
     {
         if(vangular<ang_max)
         {
             vangular=vangular+0.01;
         }
     }
     else if(Joy->buttons[1])
     {
         if(vangular>ang_min)
         {
             vangular=vangular-0.01;
         }
     }
     v.linear.x =Joy->axes[axis_lin]*vlinear; //将游戏手柄的数据乘以你想要的速度，然后发给小车
     v.angular.z =Joy->axes[axis_ang]*vangular;  

     ROS_INFO("linear:%.3lf angular:%.3lf",v.linear.x,v.angular.z);   
     //ROS_INFO("v_scale:%f ang_scale:%f",vlinear,vangular);
     pub.publish(v);  
    }
}  
int main(int argc,char** argv)  
{  
 ros::init(argc, argv, "logteleop");  
 Teleop telelog;
 ros::spin();
 return 0;  
}  

