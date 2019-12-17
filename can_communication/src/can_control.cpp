#include <iostream>
#include "can_communication/controlcan.h"
#include "can_communication/can_control.h"

#define nDeviceType  4
#define nDeviceInd  0
#define nCANInd  0

CANcommunication::CANcommunication()
{

}

CANcommunication::~CANcommunication()
{
    
}

bool  CANcommunication::Can_Open()
{
	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
	if (dwRel != 1)
	{
		std::cout << "open the device failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "open the device success" << std::endl;
		return TRUE;
	}
}

bool CANcommunication::Can_Close()
{
	dwRel = VCI_CloseDevice(nDeviceType, nDeviceInd);
	if (dwRel != 1)
	{
		std::cout << "close the device failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "close the device success" << std::endl;
		return TRUE;
	}
}

bool CANcommunication::Can_Initial()
{
	vic.AccCode = 0x80000008;
	vic.AccMask = 0xFFFFFFFF;
	vic.Filter = 1;
	vic.Timing0 = 0x00;
	vic.Timing1 = 0x1C;//500K baud freq
	vic.Mode = 0;
	dwRel = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic);
	if (dwRel != 1)
	{
		VCI_CloseDevice(nDeviceType, nDeviceInd);
		std::cout << "initial device failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "initial device sucess" << std::endl;
		return TRUE;
	}
}

bool CANcommunication::Can_Read_Device_Info()
{
	dwRel = VCI_ReadBoardInfo(nDeviceType, nDeviceInd, &vbi);
	if (dwRel != 1)
	{
		std::cout << "get device info failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "get device info success" << std::endl;
		return TRUE;
	}
}

int CANcommunication::Can_GetReceiveNum()
{
	dwRel = VCI_GetReceiveNum(nDeviceType, nDeviceInd, nCANInd);
	return dwRel;
}

bool CANcommunication::Can_ClearBuffer()
{
	dwRel = VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	if (dwRel != 1)
	{
		std::cout << "clear buffer failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "clear buffer success" << std::endl;
		return TRUE;
	}
}
bool CANcommunication::Can_Start()
{
	if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1)
	{
		VCI_CloseDevice(nDeviceType, nDeviceInd);
		std::cout << "can start failure" << std::endl;
		return FALSE;
	 }
	else
	{
		std::cout << "can start sucess" << std::endl;
		return TRUE;
	}
}

bool CANcommunication::Can_Reset()
{
	if (VCI_ResetCAN(nDeviceType, nDeviceInd, nCANInd)!=1)
	{
		std::cout << "can reset failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "can reset sucess" << std::endl;
		return TRUE;
	}
}

bool CANcommunication::Can_SendMessage(UINT SendID,UINT Data_length,BYTE SendMessage[])
{
	vco[0].ID = SendID;
	vco[0].RemoteFlag = 0;
	vco[0].ExternFlag = 0;
	vco[0].DataLen = Data_length;
	for (unsigned int j = 0; j < Data_length; j++)
		vco[0].Data[j] = SendMessage[j];
	if (VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco, 1)==-1)
	{
		std::cout << "device failure" << std::endl;
		return FALSE;
	}
	else if(VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco, 1)!=1)
	{
		std::cout << "sendmessage failure" << std::endl;
		return FALSE;
	}
	else
	{
		std::cout << "sendmessage sucess" << std::endl;
		return TRUE;

	}

}

bool CANcommunication::Can_ReceiveMessage()
{
	dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd, vci, 2500, 0);
	if (dwRel > 0)
	{
		std::cout << "receive data sucess" << std::endl;
		return TRUE;
	}
	else
	{
		std::cout << "the device is not exisit or usb lost,you can restart can device" << std::endl;
		return FALSE;
	}
}