#include "serial_node.h"


  
serial::Serial ser; //声明串口对象 
unsigned char send_data[5]={0x68,0x04,0x00,0x04,0x08};
uint8_t data;
int count=0;
unsigned char buffer[100];
unsigned char checksum=0;
unsigned char data_length=0;
unsigned char dev_id;
//回调函数 
void write_callback(const unsigned char rev_data) 
{ 
    ROS_INFO_STREAM("Writing to serial port" ); 
    ser.write(rev_data);   //发送串口数据 
} 

int Converter(const unsigned char a, const unsigned char b, const unsigned char c)
 {
  std::stringstream int2str, str2int;
  int2str << (int)a % 16 << (int)b / 16 << (int)b % 16 << (int)c / 16
          << (int)c % 16;
  int int_result;
  std::string str_result;
  int2str >> str_result;
  str2int << str_result;
  str2int >> int_result;

  if (1 == (int)a / 16) {
    int_result = -int_result;
  } else {
    int_result = abs(int_result);
  }

  return int_result;
}




int parase(unsigned char rev_data[],int len)
{
    double imu_message[9];

    imu_message[0] = (double)Converter(rev_data[1], rev_data[2], rev_data[3]) / 100.0;
    imu_message[1] = (double)Converter(rev_data[4], rev_data[5], rev_data[6]) / 100.0;
    imu_message[2] = (double)Converter(rev_data[7], rev_data[8], rev_data[9]) / 100.0;

    // attention : computing acceleration should divide 1000
    imu_message[3] = (double)Converter(rev_data[10], rev_data[11], rev_data[12]) / 100.0;
    imu_message[4] = (double)Converter(rev_data[13], rev_data[14], rev_data[16]) / 100.0;
    imu_message[5] = (double)Converter(rev_data[17], rev_data[18], rev_data[19]) / 100.0;

    imu_message[6] = (double)Converter(rev_data[20], rev_data[21], rev_data[22]) / 100.0;
    imu_message[7] = (double)Converter(rev_data[23], rev_data[24], rev_data[25]) / 100.0;
    imu_message[8] = (double)Converter(rev_data[26], rev_data[27], rev_data[28]) / 100.0;

    
    imu_msg.header.frame_id = imu_frame_id;
    imu_msg.header.stamp = ros::Time::now();

    // get orientation in quaternion form
    imu_message[0] = imu_message[0]/ 180.0 * M_PI;
    imu_message[1] = imu_message[1]/ 180.0 * M_PI;
    imu_message[2] = imu_message[2]/ 180.0 * M_PI;
    //欧拉角转四元数,类型可以不同
	geometry_msgs::Quaternion quaternion;//定义四元数
    quaternion=tf::createQuaternionMsgFromRollPitchYaw(imu_message[0],imu_message[1],imu_message[2]);
    
    imu_msg.orientation.x = quaternion.x;
    imu_msg.orientation.y = quaternion.y;
    imu_msg.orientation.z = quaternion.z;
    imu_msg.orientation.w = quaternion.w;
    // imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu_msg.orientation_covariance = {2e-8, 0, 0, 0, 2e-8, 0, 0, 0, 1.3e-7};//协方差

    // get angular velocity
    imu_msg.angular_velocity.x = imu_message[6] / 180.0 * M_PI;
    imu_msg.angular_velocity.y = imu_message[7] / 180.0 * M_PI;
    imu_msg.angular_velocity.z = imu_message[8] / 180.0 * M_PI;
    imu_msg.angular_velocity_covariance = {5e-7, 0, 0, 0, 5e-7, 0, 0, 0, 1.3e-7};

    // get linear acceleration (value * g)
    imu_msg.linear_acceleration.x = imu_message[3] * GRAVITY;
    imu_msg.linear_acceleration.y = imu_message[4] * GRAVITY;
    imu_msg.linear_acceleration.z = imu_message[5] * GRAVITY;
    imu_msg.linear_acceleration_covariance = {3.511e-5, 0, 0, 0, 3.339e-5, 0, 0, 0, 3.989e-5};

 
    return 0;

     
}
  
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //订阅主题，并配置回调函数 
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
    //指定循环的频率 
    ros::Rate loop_rate(50); 
    while(ros::ok()) 
    {   

        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n");
            data = ser.read(ser.available()); 
            if (count==0)
            {
                if(data==0x68)
                {
                    count++;
                }
            }
            else if(count==1)
            {
                data_length=data;
                checksum=checksum+data;
                count++;
            }
            else if(count==2)
            {
                dev_id=data;
                count++;
                checksum=checksum+data;
            }
            else if(count<data_length)
            {
                 buffer[count-3]=data;
                 count++;
                 checksum=checksum+data;
            }
            else if(count==data_length)
            {
                  if(checksum==data)
                  {
                      parase(buffer,data_length-3);
                      read_pub.publish(imu_msg); 
                  }
                  count=0;

            }
            
        } 

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
} 