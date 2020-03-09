#ifndef __VISION_ROBOT_H__
#define __VISION_ROBOT_H__


#include <stdint.h>
#include <stdio.h>
#include <ros/ros.h>											// 包含ROS的头文件
#include <ros/time.h>
#include <math.h>
#include <array>
#include <image_transport/image_transport.h>
#include "vision_robot/embedded_control.h"

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4

#define QMSG_KEY_1  1003
#define QMSG_KEY_2  1006
#define CAMINDEX  0

unsigned char res_buffer[IMG_WIDTH*IMG_HEIGHT] = {0};
int show_buffer[IMG_WIDTH*IMG_HEIGHT] = {0};

char ipadd[32] = "192.168.1.10";
const int tcpPort = 7842;
const int udpPort = 8180;
//QImage ImageParallax(IMG_WIDTH,IMG_HEIGHT, QImage::Format_Indexed8);

unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
unsigned char ImgBuffer_2[IMG_WIDTH*IMG_HEIGHT*4] = {0};

bool ImgBuffReady = false;
static bool bRunMode3D = true;
static bool bRunType3D = true;

void * m_Device_1 = NULL;
void * m_Device_2 = NULL;

double Q[4][4]={{1.0,0.0,0.0,-1.2058745880126953e+03},
                {0.0,1.0,0.0,-7.9784906768798828e+02},
                {0.0,0.0,0.0,2.3809219219287886090e+03},
                {0.0,0.0,-2.3434274505941843e-03,1.0638690767349501e+00}};

double b=-1.0/Q[3][2];
double f=Q[2][3];
double X0=Q[0][3];
double Y0=Q[1][3];


#endif