#ifndef EMBEDDED_CONTROL_H
#define EMBEDDED_CONTROL_H

#include <iostream>
#include "vision_robot/embeddedapi.h"

class Netclass;
class TCPclass;
class UDPclass;
class ImageFIFO;

#if defined(EMBEDDED_INTERFACE_APPLICATIONS_LIBRARY)
#  define EMBEDDED_INTERFACE_APPLICATIONSSHARED_EXPORT Q_DECL_EXPORT
#else
#  define EMBEDDED_INTERFACE_APPLICATIONSSHARED_EXPORT Q_DECL_IMPORT
#endif

class Embedded_Control
{
public:
  ///
  /// \brief Embedded_Control
  /// \param msg_id
  /// \param iWidth
  /// \param iHeight
  /// \param iChannel
  ///
    Embedded_Control(int msg_id, int iWidth, int iHeight, int iChannel);
    ~Embedded_Control();
    ///
    /// \brief Release
    /// \param m_handle
    ///
    void ctlRelease(EM_DEV_HANDLE m_handle);
    ///
    /// \brief InitTcpNetWorkParamLocal
    /// \param DeviceIpAddLists
    /// \param ip_index
    /// \return
    ///
    int ctlInitTcpNetWorkParamLocal(DeviceIpAddListsFunc* DeviceIpAddLists, int ip_index);        //init some about net,like ip | ports
    /*************************
     * decode protocol
     *************************/
    ///
    /// \brief ctlConnectTcp
    /// \return
    ///
    int ctlConnectTcp();

    void* ctlRegisterEmCallback(void *pUserParam, CallBackFunc callBackFunc);
    ///
    /// \brief ctlUnregeisterEmCallback
    /// \return
    ///
    void* ctlUnregeisterEmCallback();
    ///
    /// \brief ctlUserCallBackFunmutex
    /// \return
    ///
    void* ctlUserCallBackFunmutex();
    /*********
     * contrl cmd
     *********/
    int m_msgid;
    bool m_bDevScan;
    bool m_Ready;
    uint64_t nFrameID;
    bool m_bConnectStatus;

    ImageFIFO*   RecvImgfifo;
    ThreadParam ThPam[THREADCNT];

    CallBackFunc ctlImageCallBackFun;
    PNP_FRAME_CALLBACK_PARAM tclFrameParam[1];

    cmdParam userCmdParam;
    userCmd userCmdBuffer[CMDCNT];

    ///
    /// \brief ctlNetTcpConnectStatus
    /// \return
    ///
    int ctlNetTcpConnectStatus();
    ///
    /// \brief ctlNetTcpSendMsg
    /// \param msgBuf
    /// \param msgLength
    /// \return
    ///
    int ctlNetTcpSendMsg(char *msgBuf, long msgLength);

    bool pthread_pause;
    void pthread_suspend(char* pid);
    void pthread_resume(char* pid);
    void pthread_pause_location(void);
    ///
    /// \brief ctlNetTcpRecvMsg
    /// \param buf
    /// \param len
    /// \return
    ///
    int ctlNetTcpRecvMsg(char* buf, int len);
    ///
    /// \brief decodeTcpServerMsg
    /// \param ch_back
    ///
    void decodeTcpServerMsg(char *ch_back);

    char* emNet_RemoteIp();

    ///
    /// \brief UnDefine
    /// \return
    ///
    int rootSetCmdCalibStart();
    ///
    /// \brief rootSetCmdProjectorUploadPatterns
    /// \return
    ///
    int rootSetCmdProjectorUploadPatterns();

    int ctlCmdDeviceStart();
    int ctlCmdDeviceStop();
    int ctlCmdDeviceExit();
    int ctlCmdDeviceSW2DMode();
    int ctlCmdDeviceSW3DMode();
    int ctlCmdDeviceSWIdleMode();
    int ctlCmdDeviceProjectorOpen();
    int ctlCmdDeviceProjectorClose();
    int ctlCmdDeviceCalcModeARM();
    int ctlCmdDeviceCalcModePC();
    int ctlCmdDeviceRunOnceMode();
    int ctlCmdDeviceTrigSourceIn();
    int ctlCmdDeviceTrigSourceOut();
    int ctlCmdDeviceRunMultiMode();
    int ctlCmdDeviceLoadConfigini(const char *loadputfilepath);
    int ctlCmdDeviceRecoveConfigini();
    int ctlCmdDeviceSendConfigini(const char *sendfilepath);
    int ctlCmdDeviceGetSysStatus(uint64_t *powerOnTime, uint64_t *allFrameCnt, int32_t *frameRate, int32_t *inSideTemp, int32_t *tempStatu);
    int ctlCmdDeviceExposureTime(const uint64_t _exTime);
protected:
    bool Read_configure(char *loaclip, char *remoteip, int* device_cnt);

    int Opt_Connect_Tcp();          //connect tcp client
    int Opt_DisConnect_Net(EM_NET_TYPE netype);       //disconnect network
    int Opt_Net_Status();           //show udp/tcp connect status
    int Opt_Net_Reset_Port(int port, EM_NET_TYPE netype);
    int Opt_Cmd_Generata();
    int Opt_Cmd_Msg(char *msgtype, ...);

    void initParameter(int msg_id, int iWidth, int iHeight, int iChannel);
    void initThreadParameter();
    void initDefineParameter();
    void initMsgParameter(int MSQ_KEY);

private:
    TCPclass* tcp;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    ///
    /// \brief ctlSetCmdStatuss
    /// \param isStart
    /// \return
    ///
    int ctlSetCmdStatuss(EM_STATU_SW isStart);                       //0:stop 1:start
    ///
    /// \brief ctlSetCmdExit
    /// \return
    ///
    int ctlSetCmdExit();
    ///
    /// \brief ctlSetCmdSysMode
    /// \param sysMode
    /// \return
    ///
    int ctlSetCmdSysMode(EM_RUN_LEVEL sysMode);                   //0 :idle 1:normal  2:orignal look
    ///
    /// \brief ctlSetCmdProjectorSW
    /// \param isOpen
    /// \return
    ///
    int ctlSetCmdProjectorSW(EM_STATU_SW isOpen);       //0:close 1:open
    ///
    /// \brief ctlSetCmdCalcMode
    /// \param calcMode
    /// \return
    ///
    int ctlSetCmdCalcMode(EM_CALC_MODE calcMode);                  //0:off-line 1:on-line
    ///
    /// \brief ctlSetCmdTriggerSource
    /// \param triggerSource
    /// \return
    ///
    int ctlSetCmdTriggerSource(EM_STATU_SW triggerSource);
    ///
    /// \brief ctlSetCmdRun3DMode
    /// \param runType
    /// \return
    ///
    int ctlSetCmdRun3DMode(EM_3D_RUN_TYPE runType); //0:SIGNAL 1:CONTINUE
    ///
    /// \brief ctlSetCmdCameraExposureTime
    /// \param expTime
    /// \return
    ///
    int ctlSetCmdCameraExposureTime(const uint64_t expTime);
    ///
    /// \brief ctlSetCmdLoadConfigini
    ///
    int ctlSetCmdLoadConfigini();
    ///
    /// \brief ctlSetCmdRecovConfigini
    ///
    int ctlSetCmdRecovConfigini();
    ///
    /// \brief ctlSetCmdSendConfigini
    /// \param _filepath
    ///
    int ctlSetCmdSendConfigini(const char *_filepath);
    ///
    /// \brief ctlGetCmdSystemCurrentStatus
    /// \param sysStatus
    ///
    int ctlGetCmdSystemCurrentStatus();
};

class C_PMPAS
{
public:
    C_PMPAS();    
    ~C_PMPAS();
    //virtual int emGenarateDeviceSN() = 0;
public:

    bool  m_bIsConnect;
    bool m_bDevScan;
    bool m_bDevOpen;
    bool* m_cReady;
    int m_modeImageSizeUp;
    ThreadParam gThPamss;
    DeviceInfoScan *gui_DeviceScanInfo[32];
    /////
    /// \brief emInitudp
    /// \param dev_list
    /// \param dev_cnt
    /// \return
    ///
    int     emInitudp(DeviceInfoScan* dev_list, int dev_cnt);
    ///
    /// \brief C_PMPAS::emOpenDevice
    /// \param hDevice
    /// \param nDeviceIndex must be Consistented with the order of the list obtained
    /// \param msg_id
    ///
    void    emOpenDevice(EM_DEV_HANDLE* hDevice, int nDeviceIndex, int msg_id);
    ///
    /// \brief emCloseDevice
    /// \param m_handle
    ///
    void    emCloseDevice(EM_DEV_HANDLE* hDevice);
    ///
    /// \brief emClientNetParam
    /// \param m_handle
    /// \param nDeviceIndex
    ///
    void    emClientNetParam(EM_DEV_HANDLE hDevice, int nDeviceIndex);//////////////////
    /// \brief emRegisterImageCallback
    /// \param m_handle
    /// \param pUserParam
    /// \param callBackFun
    /// \return
    ///
    void*   emRegisterImageCallback(EM_DEV_HANDLE hDevice, void *pUserParam, CallBackFunc callBackFunc);
    ///
    ////// \brief emDevStart
    ////// \param hDevice
    ///
    int emDevStart(EM_DEV_HANDLE hDevice);
    ///
    /// \brief emDevStop
    /// \param hDevice
    ///
    int emDevStop(EM_DEV_HANDLE hDevice);
    //////////
    /// \brief emSetOutputReslutsType
    /// \param m_handle
    /// \param outputType  0:orignal image / 1:calculate resluts
    /// \return
    ///
    int     emSetOutputResultsType(EM_DEV_HANDLE m_handle, int outputType);
    ///
    /// \brief emSwitchRunMode
    /// \param m_handle
    /// \param sw_mode  2d/3d(normal)
    ///
    int    emSwitchRunMode(EM_DEV_HANDLE m_handle, int sw_mode);
    ///
    /// \brief emSetTriggerSource
    /// \param m_handle
    /// \param triggerSource
    /// \return
    ///
    int    emSetTriggerSource(EM_DEV_HANDLE m_handle, int triggerSource);
    ///
    /// \brief emSetRunMode
    /// \param pHandle
    /// \param mode
    /// \param isTriggerMode
    /// \param triggerType
    ///
    int    emSetRunMode(EM_DEV_HANDLE hDevice, EM_RUN_MODE runmode, int outputType, int outputMode, int triggerType);
    ///
    /// \brief emSetExposureTime
    /// \param hDevice
    /// \param extime
    ///
    int    emSetExposureTime(EM_DEV_HANDLE hDevice, int extime);
    ///
    /// \brief emSetOutputResultsType
    /// \param hDevice
    /// \param outputType
    /// \return
    ///
    int    emSetOutputResultsMode(EM_DEV_HANDLE hDevice, int outputMode);
    //Find feature
    ///
    /// \brief emScanDevice
    ///
    void emScanDevice();
    ///
    /// \brief emAddIpList
    /// \param ip
    /// \param ip_index
    ///
    void emAddIpList(char* ip, int ip_index);
    //multiBrodcast refranse functions
    ///
    /// \brief setMultiBrocast
    /// \param muip
    /// \param localip
    /// \return
    ///
    int setMultiBrocast(char *muip, char *localip);
    ///
    /// \brief udpRecv
    /// \param fromIP
    /// \param fromPort
    /// \param buf
    /// \param len
    /// \return
    ///
    int udpRecv(char *fromIP, int* fromPort, char *buf, int len);  
    ///
    /// \brief getDevTable
    /// \param probee
    /// \return
    ///
    int getDevTable(pDeviceInfoScan probee);
    ///
    /// \brief udpScanDevice
    /// \param MultiBrocastIP
    /// \param MultiBrocastPort
    /// \return
    ///
    int udpScanDevice(char * MultiBrocastIP, int MultiBrocastPort);
    ///
    /// \brief RegisterDataCallback
    /// \param param
    /// \param ppthis
    ///
    void RegisterDataCallback(CData param, void* ppthis, void(*pNetDataRecv)(CData pData, void* lparam));
    ///
    /// \brief emGetSysStatus
    /// \param hDevice
    /// \param allFrameCnt
    /// \param powerOnTime
    /// \param frameRate
    /// \param inSideTemp
    /// \param tempStatu
    ///
    int emGetSysStatus(EM_DEV_HANDLE hDevice, uint64_t *allFrameCnt, uint64_t *powerOnTime, int32_t *frameRate, int32_t *inSideTemp, int32_t *tempStatu);
    ///
    /// \brief emGetSysSensorInfo
    /// \param hDevice
    /// \param mac
    /// \param sn
    /// \param version
    /// \param name
    /// \return
    /// 62+151351635136
    int emGetSysSensorInfo(EM_DEV_HANDLE hDevice, int nDeviceIndex, char* ip, char *mac, char *sn, char *version, char *name);
    ///
    /// \brief emLoadConfigIniFile
    /// \param hDevice
    ///
    int emLoadConfigIniFile(EM_DEV_HANDLE hDevice, char* putfilepath);
    ///
    /// \brief emRecovConfigIniFile
    /// \param hDevice
    ///
    int emRecovConfigIniFile(EM_DEV_HANDLE hDevice);
    ///
    /// \brief emSyncLocalConfigIniFile
    /// \param hDevice
    /// \param filepath
    ///
    int emSyncLocalConfigIniFile(EM_DEV_HANDLE hDevice, char* sendfilepath);

    void doPassData(int nDeviceIndex, unsigned char* buf, unsigned int recvlen);
private:
    DeviceController devPMP[32];
    void devPMP_Collect(EM_DEV_HANDLE hDevice, int nDeviceIndex, int isRight);

    UDPclass *udp_find;
    UDPclass *udp_recvClint;
    int udpNetWorkParam(char* localip);        //init some about net,like ip | ports
    int udpConnect();
    char *udpXmltok(char *inbuf, const char *begin, const char *end, char *outbuf);
    int udpDeodeXML(char *inxml, int Localip_index, int resv_d);
    int udpEncodeXML(const char *cxml, char *nxml);
    int setDiscoveryXML(char *outxml, const char *inxml, char* setremoteip, char* setremotemac);
    void brodcastRemoteIp(char* localip, char* remoteip, char* remotemac);
};

//class EmDeviceClass : public C_PMPAS
//{
//public:
//    int emGenarateDeviceSN(){ printf("hello class\n"); return 0;};
//};

class Netclass
{
public:
    Netclass();
    ~Netclass();
public:
    char    IP[64]	;
    char    *LocalIP;
    int     PORT 	;

    void    setCurrentIpAddr(char *ipaddr, const int Port);
    int     getLocalIPs(char ips[][64]);
    int     getLocalMAC(char *MAC);

    int     sendMsg(int fd, char *buf, int len);
    int     recvMsg(int fd, char *buf, int len);
    int     disconnect(int fd);
    void    udpERROR(int error_code);
private:
    int     sys_getlocalIPs(char ips[][64]);
    int     sys_getlocalMAC(char MAC[]);

};

class TCPclass : public Netclass
{
public:
    TCPclass();
    ~TCPclass(){}
public:
    int tcpConnect();
    int socketConnected();
    int tcpSendMsg(char *buf, int len);
    int tcpRecvMsg(char *buf, int len);
    int tcpDisconnect();
private:
    //variables
    int fd_tcp ;
    struct sockaddr_in serv_tcp;
    struct sockaddr_in client_tcp;
};

class UDPclass : public Netclass
{
public:
    UDPclass();
    ~UDPclass(){}
    int udpSendMsg(char *buf, int len);
    int udpRecvMsg(char *buf, int len);
    int udpDisconnect();

    char 	fromIP[64] 	;
    char 	  toIP[64] 	;

public:
    int udpConnect();
    int tempSendudp(char *buf, int len, char *toIP, int toPort);
    int tempRecvudp(char *buf, int len, char *fromIP, int* fromPort);

    int setMultiBrocast(char *localip, char *muip);

    int createudpSocket();
    int bindudp(char *bindIP);
    int bindudp(char *bindIP, int PORT);
private:
    int fd_udp ;
    struct sockaddr_in serv_udp;
    struct sockaddr_in temp_udp;
};

class ImageFIFO
{
public:
    int 	width  				;
    int 	height 				;
    int     channel             ;
    int 	frame_sum 			;

    int 	lost_now 			;
    int 	lost_RDP_pkg_num 	;
    int 	error  				;

    unsigned int vip ;

public:
    int NaldataFIFO_Init();		//init
    int NaldataFIFO_Release();	//release
    int NaldataFIFO_PUSH(unsigned char *nal, int len);
    int NaldataFIFO_POP(unsigned char **nal, int *plen);

private:
    unsigned int nal_r_n;					//read data
    unsigned int nal_w_n;					//write data
    unsigned char nal_r_i;					//read in data
    unsigned char nal_w_i;					//write in data
    unsigned char *ppNal_buff[8];			//buffers

    pthread_mutex_t		Q_Mutex;		//thread mutex

};

//////////////////////////////////////////////////////////////////////////////////////////////////////
#endif // EMBEDDED_CONTROL_H


