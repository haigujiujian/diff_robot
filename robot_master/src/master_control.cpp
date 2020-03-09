#include "master_control.h"

system_control::system_control()
{
    
}

system_control::~system_control()
{
    

}

system_control  system_master;

int main(int argc, char** argv)
{
     char start_flag;
     
     ros::init(argc, argv, "master_control");	//初始化节点
	 ros::Time::init();
     ros::Rate loop_rate(20);
     ros::NodeHandle nh;
     ros::NodeHandle private_nh("communication_ns");
     //std::cout << private_nh.getNamespace() << std::endl;
     //ros::Duration(1.0).sleep();
     std::cout<<"Are you want to start system y/n"<<std::endl;
     std::cin>>start_flag;
     if(start_flag=='y')
     {
        while(ros::ok())
        {
        
     
            /* code for loop body */
            private_nh.getParam("liftmotor_flag",system_master.liftmotor_flag);
            private_nh.getParam("liftmotor_level",system_master.liftmotor_level);
            private_nh.getParam("nav_flag",system_master.nav_flag);
            private_nh.getParam("nav_pos",system_master.nav_pos);
            private_nh.getParam("scan_flag",system_master.scan_flag);
            if(system_master.nav_flag==0&system_master.nav_pos==0)
            {
                private_nh.setParam("nav_flag",1);
                private_nh.setParam("nav_pos",1);
            }
            else if(system_master.nav_flag==0&system_master.nav_pos==1)
            {
                private_nh.setParam("liftmotor_flag",1);
                private_nh.setParam("liftmotor_level",1);
            }
            else if(system_master.nav_pos==1&system_master.liftmotor_level==1&system_master.liftmotor_flag==0)
            {
                private_nh.setParam("scan_flag",1);
            }
            ROS_INFO("liftmotor_flag:%d,movingmotor_flag:%d",system_master.liftmotor_flag,system_master.scan_flag);
            ros::spinOnce();
            loop_rate.sleep();
        }
     }
     
     
       
}