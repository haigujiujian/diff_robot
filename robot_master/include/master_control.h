#ifndef MASTER_CONTROL_H
#define MASTER_CONTROL_H 

#include<stdint.h>
#include<stdio.h>
#include <ros/ros.h>											// 包含ROS的头文件
#include <ros/time.h>

class system_control
{
public:
    system_control();
    ~system_control();
    int liftmotor_flag ;
    int nav_flag,nav_pos;
    int liftmotor_level;
    int scan_flag;

private:
    /* data */
};



#endif