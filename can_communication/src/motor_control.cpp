#include<iostream>
#include"can_communication/motor_control.h"
#include"can_communication/can_control.h"

  CANcommunication CAN_com1;

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
	speed =   (512*Motor_Speed) ;
	speed = (speed*encoder_num)/1875;
	speed = int(speed);
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


bool Motor_Control::Motor_Feedback()
{
	BYTE data[2];
	CAN_com1.Can_SendMessage(0x80, 0, data);
	CAN_com1.Can_ReceiveMessage();
	if (CAN_com1.vci[0].ID == 0x181)
	{
		std::cout << "I have receive data" << std::endl;
		return TRUE;

	}
	else
	{
		std::cout << "I don't receive data" << std::endl;
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

int Motor_Control::ByteHex2Int(BYTE *data_vec, const int &data_vec_len)
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
}