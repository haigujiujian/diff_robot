#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include"can_communication/controlcan.h"
#define INT16 unsigned int
class Motor_Control
{
public:
	Motor_Control();
	~Motor_Control();

public:
	bool Motor_PDO_Open();
	void Motor_Speed_Control(INT16 MotorID, int Motor_Speed, BYTE WorkMode, BYTE CONTROL_Word[2]);
	bool Motor_Feedback();
	void Dec2HexVector(BYTE *data_vec, const int &dec_value, const int &len);
	int ByteHex2Int(BYTE *data_vec, const int &data_vec_len);

private:
	int MotorID_Num1=0x201;
	int MotorID_Num2=0x202;
	BYTE PDO_Open[2];	
	int encoder_num = 65536;


};

#endif