#ifndef EMBEDDEDAPI_H
#define EMBEDDEDAPI_H

#if defined(_WIN32)
//windows
    #include "stdafx.h"

#else
//linux
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netdb.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <ctype.h>
    #include <errno.h>
    #include <malloc.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <sys/ioctl.h>
    #include <stdarg.h>
    #include <fcntl.h>

    #include <net/if.h>
    #include <net/if_arp.h>
//    #include <linux/tcp.h>
    #include <pthread.h>
    #include <signal.h>
    #include <linux/spi/spidev.h>
    #include <vector>

    #include <setjmp.h>
    #include <netinet/tcp.h>

    #include <sys/types.h>
    #include <sys/ipc.h>
    #include <sys/msg.h>
    #include <uuid/uuid.h>
#endif
//////////////////////////////////////////////////////////////////////////
//  类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//  VS2010之前的版本中都不包含此文件,所以在此需要重定义
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
    #ifndef _STDINT_H
        #ifdef _MSC_VER // Microsoft compiler
            #if _MSC_VER < 1600
                typedef __int8            int8_t;
                typedef __int16           int16_t;
                typedef __int32           int32_t;
                typedef __int64           int64_t;
                typedef unsigned __int8   uint8_t;
                typedef unsigned __int16  uint16_t;
                typedef unsigned __int32  uint32_t;
                typedef unsigned __int64  uint64_t;
            #else
                // In Visual Studio 2010 is stdint.h already included
                #include <stdint.h>
            #endif
        #else
            // Not a Microsoft compiler
            #include <stdint.h>
        #endif
    #endif
#else
    // Linux
    #include <stdint.h>
#endif


//------------------------------------------------------------------------------
//  操作系统平台定义
//------------------------------------------------------------------------------

#include <stddef.h>

#ifdef WIN32
    #ifndef _WIN32
        #define _WIN32
    #endif
#endif

#ifdef _WIN64
    #define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef _WIN32
    #include <Windows.h>
    #define EM_DLLIMPORT   __declspec(dllimport)
    #define EM_DLLEXPORT   __declspec(dllexport)

    #define EM_STDC __stdcall
    #define EM_CDEC __cdecl

    #if defined(__cplusplus)
        #define EM_EXTC extern "C"
    #else
        #define EM_EXTC
    #endif
#else
    // remove the None #define conflicting with GenApi
    #undef None
    #if __GNUC__>=4
        #define EM_DLLIMPORT   __attribute__((visibility("default")))
        #define EM_DLLEXPORT   __attribute__((visibility("default")))

        #if defined(__i386__)
            #define EM_STDC __attribute__((stdcall))
            #define EM_CDEC __attribute__((cdecl))
        #else
            #define EM_STDC
            #define EM_CDEC
        #endif

        #if defined(__cplusplus)
            #define EM_EXTC extern "C"
        #else
            #define EM_EXTC
        #endif
    #else
        #error Unknown compiler
    #endif
#endif

#ifdef EM_GALAXY_DLL
    #define EM_DLLENTRY EM_EXTC EM_DLLEXPORT
#else
    #define EM_DLLENTRY EM_EXTC EM_DLLIMPORT
#endif

/*********************************
 * cmd
 * *******************************************/
//pcl
#define GUI_CMD_OUTPUT_CAL_TYPE     		"SetCalOutPutType" //设置输出计算结果类型 [cmdx]x:0 视差 x:1 点云

//calibrations
#define GUI_CMD_CALIB_TO_CALIB_MODE    		"CmdToClibModal" 	//切换至在线标定模式
#define GUI_CMD_CALIB_ACTIVE_GETIMAGE     	"CmdStartGetImg"	//开始获取图片
#define GUI_CMD_CALIB_ACTIVE_CALC	     	"CmdStartCalC"		//开始计算
#define GUI_CMD_CALIB_SET_BOARDWIDTH	    "SetBoardWidth"		//设置标定板宽[cmdx]
#define GUI_CMD_CALIB_SET_BOARDHIGHT	    "SetBoardHight"		//设置标定板高[cmdx]
#define GUI_CMD_CALIB_SET_PATTERNNUM	    "SetPatternNum"		//设置标定照片张数[cmdx]
#define GUI_CMD_CALIB_SET_SQUARESIZE	    "SetSquareSize"		//设置方格大小[cmdx]
#define GUI_CMD_CALIB_ACTIVE_SHOWRESULT 	"CmdShowResult"		//输出计算结果
#define GUI_CMD_CALIB_SET_EXPOSURETIME      "SetExposureTime"	//设置曝光时间[cmdx]
#define GUI_CMD_CALIB_TO_OFFLINE_MODE		"CmdToCalOffLineModal" //切换离线标定模式

//camera
#define GUI_CMD_CAMERA_GET_MINEXPOSURE      "CmdGetMinExposure"	//获取最小曝光时间
#define GUI_CMD_CAMERA_ACTIVE_TRIGGERONCE   "CmdTriggerOnce"	//触发一次 一组patterns
#define GUI_CMD_CAMERA_SET_TRIGGERTIMES     "SetTriggerTimes"	//触发次数[cmdx]
#define GUI_CMD_CAMERA_SET_TRIGGEREACHTIME  "SetTriggerEachTime"//触发间隔[cmdx]需要大于min
#define GUI_CMD_CAMERA_SET_TRIGGER_MODE    	"SetTriggerMode"	//触发模式 [cmdx]0:hardware 1:software

//projector
#define GUI_CMD_PROJECTOR_ACTIVE_CONNECT        	"CmdConnectPorjector" //链接投影
#define GUI_CMD_PROJECTOR_ACTIVE_RESET       		"CmdReConnectPorjector"//重连投影
#define GUI_CMD_PROJECTOR_ACTIVE_VALIDATE    		"CmdValidate"		//投影检测是否有效
#define GUI_CMD_PROJECTOR_ACTIVE_STOP    			"CmdProjectorStop"	//停止投影
#define GUI_CMD_PROJECTOR_ACTIVE_START    			"CmdProjectorStart"	//打开投影
#define GUI_CMD_PROJECTOR_ACTIVE_PAUSE    			"CmdProjectorPause"	//暂停投影
#define GUI_CMD_PROJECTOR_TO_UPLOAD_MODE			"CmdProjectorUpLoad"//烧写patterns

//net
#define GUI_CMD_NET_SET_RESLUTIONRADIA       		"SetReslutionRadia" //设置图片分辨率[cmdx]

//run
#define GUI_CMD_CONTROL_SET_TRIGGER_SOURCE    			"SetSysTriggerSource"	//触发模式 [cmdx]0:outside 1:inside
#define GUI_CMD_CONTROL_SET_RUNLEVEL					"CmdRunLevel"	//设置运行等级
#define GUI_CMD_CONTROL_ACTIVE_START					"CmdStartRun"	//开始
#define GUI_CMD_CONTROL_ACTIVE_STOP						"CmdStopRun"	//停止
#define GUI_CMD_CONTROL_ACTIVE_CAL						"CmdStartCal"	//FPGA计算
#define GUI_CMD_CONTROL_ACTIVE_SRC						"CmdStartSrc"	//原图
#define GUI_CMD_CONTROL_SET_CAL_TYPE					"SetCalType"	//连续模式还是单次模式[cmdx]0:单次，1：连续
#define GUI_CMD_CONTROL_SET_SRC_LEFT_ADD        		"SetSrcLeftAdd"	//原图左边显示加[cmdx]
#define GUI_CMD_CONTROL_SET_SRC_RIGHT_ADD       		"SetSrcRightAdd"//原图右边显示加[cmdx]
#define GUI_CMD_CONTROL_SET_CAL_M_ADD					"SetCalMadd"	//FPGA计算结果通道加[cmdx]
#define GUI_CMD_CONTROL_SET_CAL_OFFORON         		"SetCalOffOrOn"	//点云在线或者离线计算[cmdx]
#define GUI_CMD_CONTROL_ACTIVE_JUMPROUTER          		"CmdJumpRouter"	//跳过路由配置即发现
#define GUI_CMD_CONTROL_GET_RUNTIME						"GetRunTime"	//获取系统运行时间
#define GUI_CMD_CONTROL_GET_FPGATIME					"GetFpgaTime"	//获取系统成像次数
#define GUI_CMD_CONTROL_GET_TEMPRATURE                  "GetTemprature" //获取温度
#define GUI_CMD_CONTROL_GET_FRAMERATE                   "GetFrameRate"  //获得帧率
#define GUI_CMD_CONTROL_ACTIVE_RECOV_CONF				"CmdRecovPara"	//配置文件恢复出厂设置
#define GUI_CMD_CONTROL_ACTIVE_LOAD_CONF				"CmdLoadPara"	//读取配置文件
#define GUI_CMD_CONTROL_ACTIVE_SYNC_CONF				"CmdSyncPara"	//同步配置文件，加载用户配置文件

#define GUI_CMD_CONTROL_TO_2DCAMERA_MODE 				"CmdProCamerLookMode" //进入2D模式
#define GUI_CMD_CONTROL_TO_NORMAL_MODE         			"CmdRunNormalMode"//进入正常模式
#define GUI_CMD_CONTROL_TO_IDLE_MODE         			"CmdRunIdleMode"//进入空闲模式

#define GUI_CMD_CONTROL_EXIT							"CmdExitApp"	//退出APP

#define MAX_MSG_LEN 1024
//#define MSQ_KEY 1005

#define THREADCNT    4
#define SocketAddressSize sizeof(struct sockaddr_in)

#define MAX_IP_CNT 32
#define NET_CARD_NAME           "enp3s0"
#define CMDCNT 6
#define MULTIBROADCAST_IP       "239.255.255.250"
#define MULTIBROADCAST_PORT     5702
#define MULTI_SPACI_FROM_PORT_NUM    63
#define MULTI_SPACI_TO_PORT_NUM      62
//------------------------------------------------------------------------------
//  错误码定义
//------------------------------------------------------------------------------
typedef enum EM_STATUS_LIST
{
    EM_STATUS_SUCCESS                       =  0,           ///< 成功
    EM_STATUS_ERROR                         = -1,           ///< 不期望发生的未明确指明的内部错误
    EM_STATUS_CAMER_NOT_FOUND_TL            = -2,           ///< 找不到TL库
    EM_STATUS_CAMER_NOT_FOUND_DEVICE        = -3,           ///< 找不到设备
    EM_STATUS_CAMER_OFFLINE                 = -4,           ///< 当前设备为掉线状态
    EM_STATUS_INVALID_PARAMETER             = -5,           ///< 无效参数,一般是指针为NULL或输入的IP等参数格式无效
    EM_STATUS_INVALID_HANDLE                = -6,           ///< 无效句柄
    EM_STATUS_INVALID_CALL                  = -7,           ///< 无效的接口调用,专指软件接口逻辑错误
    EM_STATUS_INVALID_ACCESS                = -8,           ///< 功能当前不可访问或设备访问模式错误
    EM_STATUS_NEED_MORE_BUFFER              = -9,           ///< 用户申请的buffer不足:读操作时用户输入buffersize小于实际需要
    EM_STATUS_ERROR_TYPE                    = -10,          ///< 用户使用的FeatureID类型错误，比如整型接口使用了浮点型的功能码
    EM_STATUS_OUT_OF_RANGE                  = -11,          ///< 用户写入的值越界
    EM_STATUS_NOT_IMPLEMENTED               = -12,          ///< 当前不支持的功能
    EM_STATUS_NOT_INIT_API                  = -13,          ///< 没有调用初始化接口
    EM_STATUS_TIMEOUT                       = -14,          ///< 超时错误
    EM_STATUS_PROJECTOR_ERROR               = -15,          ///< projector connect error
    EM_STATUS_REMAPLIST_UNLOAD              = -16,          ///< remap upload error

}EM_STATUS_LIST;
typedef int32_t EM_STATUS;

typedef enum EM_THREAD_ID_NUM
{
    EM_ENUM_ID_CALLBACK           = 0,               ///<引脚0
    EM_ENUM_ID_TCP_RECV           = 1,               ///<引脚1
    EM_ENUM_ID_UDP_RECV           = 2,               ///<引脚2
    EM_ENUM_ID_TCP_RECV_RETURE    = 3
} EM_THREAD_ID_NUM;

typedef enum EM_NET_TYPE
{
    EM_NET_TCP = 0,
    EM_NET_UDP = 1,
}EM_NET_TYPE;

typedef enum EM_STATU_SW
{
    EM_OFF = 0,
    EM_ON  = 1
}EM_STATU_SW;
typedef EM_STATU_SW EM_CALC_MODE ;
typedef EM_STATU_SW EM_3D_RUN_TYPE;

typedef enum EM_RUN_LEVEL
{
    EM_IDLE        = 0,
    EM_NORMAL_3D   = 1,
    EM_2D_ORGLOOK  = 2

}EM_RUN_LEVEL;

typedef enum _EM_RUN_MODE
{
    EM_RUN_2D_MODE = 0,
    EM_RUN_3D_MODE = 1
}EM_RUN_MODE;

//------------------------------------------------------------------------------
//  标准C API功能函数定义
//------------------------------------------------------------------------------
#define EM_API EM_EXTC EM_STATUS EM_STDC

typedef struct cmdParam
{
    char cmd_head[10];
    char cmd_type[6];
    char cmd_ip[32];
    char cmd_port[8];
    char cmd_data[1024];
    char cmd_tril[8];
}cmdParam;

typedef struct Cmd
{
    int inum;
    void* pdata;
}userCmd;

/******************
 * define threadparam
 ******************/
typedef struct ThreadParam
{
    char name[64] ;

    pthread_t 		id   ;	// id
    pthread_attr_t 	attr ;	// attr

    int begin 	;
    int running ;
    int end 	;

    int err_code    ;
    char *err_flag  ;

}ThreadParam;

/* @struct: packeg header
 *
 * */
struct pak_header
{
    char	ID[8] ;
    int		width ;
    int		height ;
    int		len	;
    int		index ;
    int		arg1 ;
    int		arg2 ;
    int		m ;		// last packeg
    int     type;   //0:left 1:right 2:calib_result 3:calculate_result
    int     recv;
};

typedef struct MSGQUEUE
{
    long msgType;
    char msgText[MAX_MSG_LEN];

}Msg;

/******************
 * define CALLBACK_PARAM
 ******************/
typedef struct PNP_FRAME_CALLBACK_PARAM
{
    void* pUserParam    ;
    void* pImgBuf ;
    uint64_t nFrameID   ;
    uint64_t pBufferSize;
    uint32_t *reserved  ;

}PNP_FRAME_CALLBACK_PARAM   ;

typedef void (EM_STDC* CallBackFunc)(PNP_FRAME_CALLBACK_PARAM *pFrame);     //usr inter to deal buffer

typedef void* EM_DEV_CLASS;

typedef struct  PROBE_
{
    int				Video_Port;	//VIDEO PORT
    int				CMD_Port;	//CMD PORT
    int				Ctrl_Port;	//CTRL PORT
    int				Res_Port;	//RES PORT
    char			IP[32];
    char			MAC[32];
    char			Types[32];
    char			Name[64];
    char			SN[64];
    char			Version[64];
}UDP_PROB;

typedef struct LOCAL_PROBE_
{
    UDP_PROB probe_[32];
    int      localip_index;
    int      localip_num;
}LOCAL_PROBE_;

typedef struct USER_PROBE
{
    char			IP[32];
    char			MAC[32];    
    char			SN[64];
    char			Version[128];
    char			Name[64];
    char            lIP[64];
}DeviceInfoScan;

typedef struct DevCtl
{
    void* devHandle;
    DeviceInfoScan *devInfo;
}DeviceController;

typedef DeviceInfoScan*  pDeviceInfoScan;

typedef struct Remap_IP_LIST
{
    char ScanIP_List[32][64];
    char LocalIP_List[32][64];
    bool used_ok[32];
    int  cnt;
}DeviceIpAddListsFunc;

typedef struct CData
{
    //C_PMPAS* ppthis;
    char remoteip[64];
    int  remoteport;
    int  recvLen;
    unsigned char* recv_pack;
}CData;

typedef void* EM_DEV_HANDLE;///< 设备句柄，通过EMOpenDevice获取，通过此句柄进行控制与采集
typedef void* EM_EVENT_CALLBACK_HANDLE;///< 设备事件回调句柄，注册设备相关事件回调函数，比如设备掉线回调函数
#endif // EMBEDDEDAPI_H
