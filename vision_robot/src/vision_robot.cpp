#include "vision_robot/vision_robot.h"



static void OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM*pFrame)
{
    int recvBufID_Old=0;
    int recvBufID=pFrame->nFrameID;
    if(recvBufID_Old!=recvBufID)
    {
        ImgBuffReady=true;
        recvBufID_Old=pFrame->nFrameID;
        memcpy(res_buffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
    }
}

 int main(int argc, char** argv)
 {  
    C_PMPAS  vision_control;
   /* ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    sensor_msgs::ImagePtr 
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);*/
    vision_control.emScanDevice();
    vision_control.emAddIpList(ipadd,CAMINDEX);
    vision_control.emOpenDevice(&m_Device_1, CAMINDEX, QMSG_KEY_1);
    vision_control.emClientNetParam(m_Device_1,CAMINDEX);
    vision_control.emRegisterImageCallback(m_Device_1, (void*)NULL, OnTestCallBackFun);
    vision_control.emSetOutputResultsMode(m_Device_1,EM_ON);
    vision_control.emSwitchRunMode(m_Device_1,EM_ON);
    vision_control.emDevStart(m_Device_1);
    ros::Rate loop_rate(5);
    while(1)
    {
        if(ImgBuffReady)
        {
            int *buf_img32=(int*) res_buffer;
           // msg =res_buffer;
            /*for(int i=1;i<1536;i++)
            {
                for(int j=256;j<2000;j++)
                {
                    int tempvalue=buf_img32[i*IMG_WIDTH+j];
                    tem_point.z=0.0;
                    tem_point.x=0.0;
                    tem_point.y=0.0;
                    
                    if(tempvalue!=0)
                    {
                        double thisDispMap=tempvalue+preDisprity;
                        double thisDispMap_f=thisDispMap/pow(2,12)+OFFSET;
                        tem_point.z=b*f/thisDispMap_f;
                        tem_point.x=tem_point.z*(j+target_L_Rect_x+X0)/f;
                        tem_point.y=tem_point.z*(i+target_L_Rect_y+Y0)/f;
                    }
                    //cloud->push_back(tem_point);
                }

            }*/
    
        }
        ImgBuffReady=false;
        /*pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();*/
    }

     vision_control.emDevStop(m_Device_1);
   
     return 0;
 }