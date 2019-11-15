#ifndef _CAN_CONTROL_H
#define _CAN_CONTROL_H

class CANcommunication
{
public:
	CANcommunication();

	~CANcommunication(); 
	
public:
	bool Can_Open();
	bool Can_Close();
	bool Can_Initial();
	bool Can_Read_Device_Info();
	int  Can_GetReceiveNum();
	bool Can_ClearBuffer();
	bool Can_Start();
	bool Can_Reset();
	bool Can_SendMessage(UINT SendID,UINT Data_length ,BYTE SendMessage[]);
	bool Can_ReceiveMessage();
	VCI_CAN_OBJ vci[2500];

private:
	
    DWORD dwRel;
	VCI_INIT_CONFIG vic;
	VCI_BOARD_INFO vbi;
	VCI_CAN_OBJ vco[1];
	
	


};

#endif