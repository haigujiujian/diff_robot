#ifndef _SERIAL_NODE_
#define _SERIAL_NODE_

#include <string>
#include <sstream>
#include "ros/ros.h" 
#include "serial/serial.h"  //ROS已经内置了的串口包 
#include "std_msgs/String.h" 
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h" //amcl_pose
#include "tf/transform_datatypes.h" //转换函数头文件
#include "tf/tf.h"
#include "sensor_msgs/Imu.h"

#define GRAVITY 9.80665

std::string imu_frame_id;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub;
#endif