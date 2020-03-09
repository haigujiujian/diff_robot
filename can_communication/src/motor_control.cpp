#include<stdio.h>
#include<iostream>
#include"can_communication/motor_control.h"
#include"can_communication/can_control.h"

  CANcommunication CAN_com1;
  BYTE Control_Word_Stop[2]={0x06,0x00} ;
  union Control_value
  {
	  BYTE modify_value[4];
	  int exchange_value;
  };

Motor_Control::Motor_Control()
{
    CAN_com1.Can_Open();
	CAN_com1.Can_Initial();
	CAN_com1.Can_Start();
	PDO_Open[0] = 0x01;
	PDO_Open[1] = 0x00;
}

Motor_Control::~Motor_Control()
{
	this->Motor_Speed_Control(0x201, 0, 0x03,Control_Word_Stop);
	this->Motor_Speed_Control(0x202, 0, 0x03,Control_Word_Stop);
	this->Motor_Speed_Control(0x202, 0, 0x03,Control_Word_Stop);
	this->Motor_Speed_Control(0x202, 0, 0x03,Control_Word_Stop);
	CAN_com1.Can_Close();
}


bool Motor_Control::Motor_PDO_Open()
{
	
	if (CAN_com1.Can_SendMessage(0x00, 2, PDO_Open))
	{
		std::cout << "PDO start  success" << std::endl;
		return TRUE;
	}
	else
	{
		std::cout << "PDO start failure" << std::endl;
		return FALSE;
	}
}

void Motor_Control::Motor_Speed_Control(INT16 Motor_RPDO_ID, int Motor_Speed,BYTE WorkMode,BYTE CONTROL_Word[2])
{
	
	BYTE Send_Message[7];
	BYTE Speed_Message[4];
	double speed;
	speed =   (512.0*Motor_Speed*encoder_num)/1875.0 ;
	Dec2HexVector(Speed_Message, speed, 4);
	Send_Message[0] = CONTROL_Word[0];
	Send_Message[1] = CONTROL_Word[1];
	Send_Message[2] = WorkMode;
	Send_Message[3] = Speed_Message[0];
	Send_Message[4] = Speed_Message[1];
	Send_Message[5] = Speed_Message[2];
	Send_Message[6] = Speed_Message[3];
	if (CAN_com1.Can_SendMessage(Motor_RPDO_ID, 7, Send_Message) == TRUE)
	{
		std::cout << "speed control send sucess" << std::endl;
	}
	else
	{
		std::cout << "speed control send failure" << std::endl;
	}
}

void Motor_Control::Motor_Mode_Control(INT16 Motor_RPDO_ID,BYTE WorkMode,BYTE CONTROL_Word[2])
{
	BYTE Send_Message[3];
	Send_Message[0]=WorkMode;
	Send_Message[1]=CONTROL_Word[0];
	Send_Message[2]=CONTROL_Word[1];
	if (CAN_com1.Can_SendMessage(Motor_RPDO_ID, 3, Send_Message) == TRUE)
	{
		std::cout << "mode control send sucess" << std::endl;
	}
	else
	{
		std::cout << "mode control send failure" << std::endl;
	}
}
void Motor_Control::Motor_Lift_Control(INT16 Motor_RPDO_ID,int Target_Position,int Lift_Trapezoid_Speed )
{
    BYTE Send_Message[8];
	union Control_value position ,speed;
	position.exchange_value=Target_Position;
	speed.exchange_value=(512.0*Lift_Trapezoid_Speed*encoder_num)/1875.0;
    Send_Message[0]=position.modify_value[0];
	Send_Message[1]=position.modify_value[1];
	Send_Message[2]=position.modify_value[2];
	Send_Message[3]=position.modify_value[3];
	Send_Message[4]=speed.modify_value[0];
	Send_Message[5]=speed.modify_value[1];
	Send_Message[6]=speed.modify_value[2];
	Send_Message[7]=speed.modify_value[3];
	if (CAN_com1.Can_SendMessage(Motor_RPDO_ID, 8, Send_Message) == TRUE)
	{
		std::cout << "lift control send sucess" << std::endl;
	}
	else
	{
		std::cout << "lift control send failure" << std::endl;
	}

}



bool Motor_Control::Motor_Feedback()
{
	BYTE data[2];
    CAN_com1.Can_SendMessage(0x80, 0, data);
	if (CAN_com1.Can_ReceiveMessage())
	{
		//std::cout << "I have receive data" << std::endl;
	
		
		for(int i=0;i<4;i++)
		{
			if(CAN_com1.vci[i].ID==0x282)
			{
				speed_change.real_time_speed[0]=CAN_com1.vci[i].Data[0];
				speed_change.real_time_speed[1]=CAN_com1.vci[i].Data[1];
                speed_change.real_time_speed[2]=CAN_com1.vci[i].Data[2];
				speed_change.real_time_speed[3]=CAN_com1.vci[i].Data[3];

				/*for(i=0;i<4;i++)
				{
					std::cout<<std::to_string(speed_change.real_time_speed[i])<<std::endl;
				}*/
				left_realtime_Speed=-((speed_change.real_speed*1875.0)/(encoder_num*512.0));
				if(left_realtime_Speed<5&left_realtime_Speed>-5)
				{
					left_realtime_Speed=0;
				}				
				std::cout<<"left_real_speed:"<<std::to_string(left_realtime_Speed)<<std::endl;
				

			}
			else if(CAN_com1.vci[i].ID==0x281)
			{
                speed_change.real_time_speed[0]=CAN_com1.vci[i].Data[0];
				speed_change.real_time_speed[1]=CAN_com1.vci[i].Data[1];
                speed_change.real_time_speed[2]=CAN_com1.vci[i].Data[2];
				speed_change.real_time_speed[3]=CAN_com1.vci[i].Data[3];
				/*for(i=0;i<4;i++)
				{
					std::cout<<std::to_string(speed_change.real_time_speed[i])<<std::endl;
				}*/
				right_realtime_Speed=(speed_change.real_speed*1875.0)/(encoder_num*512.0);
				if(right_realtime_Speed<5&right_realtime_Speed>-5)
				{
					right_realtime_Speed=0;
				}
				std::cout<<"right_real_speed:"<<std::to_string(right_realtime_Speed)<<std::endl;
				
			}
			
			if(i==4)
			{
				return TRUE;
			}
		}

	}
	else
	{
		std::cout<<CAN_com1.vci[0].ID <<std::endl;
		//std::cout << "I don't receive data" << std::endl;
		return FALSE;
	}

}

void Motor_Control::Dec2HexVector(BYTE *data_vec, const int &dec_value, const int &len)
{
	for (int i = 0; i < len; i++)
	{
		data_vec[i] = (((int)dec_value >> (i * 8)) & 0xff);
	}
}

/*int Motor_Control::ByteHex2Int(BYTE *data_vec, const int &data_vec_len)
{
	int result = 0;
	int sign_judger = 0x80;
	sign_judger = sign_judger << ((data_vec_len - 1) * 8);
	for ( int i = 0; i < data_vec_len; i++)
	{
		result += (data_vec[i] << (i * 8));
		
	}
	if (sign_judger&&result)
	{
		result = -1 * (~(result - 1));
	}
	return result;
}*/

