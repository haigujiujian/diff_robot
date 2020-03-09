#include "vision_robot/embedded_control.h"

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CHANNEL 4

#define bLOCKUINT   4096

#define HANDLE_VALIDATE     \
    if(hDevice == NULL)    \
        exit(1);

#define pHANDLE_VALIDATE     \
    if(hDevice == NULL)    \
        return (void*)0;

void* threadCallBackToUser(void* lpParam) ;
void* threadTcpSendCommand(void* lpParam)  ;
//void* Thread_Decode_UDP_User(void* lpParam)  ;
void* threadDecodegUdpUser(void* lpParam);
void* threadDecodegTcpMsg(void* lpParam);

static void OnNetDataCallbackFun(CData pData, void *lparam);

const char C_ZERO = '\0';
const char I_ZERO = 0;
static const char initDefaultIpAdd[64] = "192.168.0.10";
static const int initDefaultPortUdp = 8180;
static const int initDefaultPortTcp = 7842;
static const int recvsize = 512;

char gIpAdd[64] = {0};
ImageFIFO  gRecvImgfifo[10];
unsigned char g_bufD[1600] = {0};
int g_index = 0;
unsigned char *buf_img[10];

static uint64_t gpowerOnTime;
static uint64_t gallFrameCnt;
static int32_t gframeRate;
static int32_t ginSideTemp;
static int32_t gtempStatu;

static int m_img_ch = 4;
static char iniPath[512] = "./";
/////////////////////////////////////////////
DeviceIpAddListsFunc gDeviceIpAddListsFunc;
DeviceInfoScan gDeviceInfoScan[32];
int  localIP_num;
int  remoteIP_num;
char udpLocalips[32][64];
LOCAL_PROBE_ localprobe_[32];

pthread_mutex_t thread_mutex;

Embedded_Control::Embedded_Control(int msg_id, int iWidth = 2048, int iHeight = 1536, int iChannel = 4)
  :RecvImgfifo()
{
    initThreadParameter();
    initDefineParameter();
    initMsgParameter(msg_id);

    RecvImgfifo->width = iWidth;
    RecvImgfifo->height = iHeight;
    RecvImgfifo->channel = iChannel;
}

Embedded_Control::~Embedded_Control()
{

}

void Embedded_Control::ctlRelease(EM_DEV_HANDLE m_handle)
{
    Embedded_Control* pthis = (Embedded_Control*)m_handle;

    pthis->pthread_resume(0);
    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running = 0;
    while(pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].end != 1)
    {
        pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running = 0;
        usleep(100);
    }
    pthis->ctlSetCmdExit();//quit the tcp-msg thread
    usleep(100);

    pthis->ThPam[EM_ENUM_ID_CALLBACK].running = 0;
    while(pthis->ThPam[EM_ENUM_ID_CALLBACK].end != 1)
    {
        pthis->ThPam[EM_ENUM_ID_CALLBACK].running = 0;
        usleep(100);
    }
    pthis->ctlUnregeisterEmCallback();
    pthis->tcp->tcpDisconnect();
    delete pthis->tcp;
    delete pthis->RecvImgfifo;
}

void Embedded_Control::initParameter(int msg_id, int iWidth = 2048, int iHeight = 1536, int iChannel = 4)
{
    initThreadParameter();
    initDefineParameter();
    initMsgParameter(msg_id);

    RecvImgfifo->width = iWidth;
    RecvImgfifo->height = iHeight;
    RecvImgfifo->channel = iChannel;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBLE STATIC FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int ReSizeMemery(int resizeOfmemery,unsigned char** buff,int TypeMem)
{
    void* temp = NULL;
    if(TypeMem == 0)
    {
        temp = realloc(*buff,resizeOfmemery * sizeof(unsigned char));
    }
    else if(TypeMem == 1)
    {
        temp = realloc(*buff,resizeOfmemery * 2 * sizeof(unsigned char));
    }
    if(temp == NULL)
        return -1;
    else
        *buff = (unsigned char*)temp;

    return 0;
}

#if 0
static void funswitch(unsigned char * src1, unsigned char* src2)
{
    *src1 = *src1 ^ *src2;
    *src2 = *src2 ^ *src1;
    *src1 = *src1 ^ *src2;
}
#endif

static void MyDelay(uint64_t delayT)
{
    usleep(delayT * 1000);
}

void* threadCallBackToUser(void* lpParam)
{
    Embedded_Control* pthis = (Embedded_Control*)lpParam;
    pthis->ThPam[EM_ENUM_ID_CALLBACK].begin = 1;
    pthis->ThPam[EM_ENUM_ID_CALLBACK].running = 1;

    while(pthis->ThPam[EM_ENUM_ID_CALLBACK].running == 1)
    {
        //pthis->pthread_pause_location();
        if(pthis->m_Ready)
        {
            pthis->ctlUserCallBackFunmutex();
        }
        else
        {
            usleep(1000);
            continue;
        }
        usleep(1000);
    }

    pthis->ThPam[EM_ENUM_ID_CALLBACK].begin     = 0;
    pthis->ThPam[EM_ENUM_ID_CALLBACK].end       = 1;
    pthis->ThPam[EM_ENUM_ID_CALLBACK].running   = 0;
    pthis->ThPam[EM_ENUM_ID_CALLBACK].err_code  = 0;
    pthread_exit(NULL);
    return 0;
}

void* threadDecodegUdpUser(void* lpParam)
{
    C_PMPAS* pthis = (C_PMPAS*)lpParam;
    pthis->gThPamss.begin = 1;

    // packeg header
    pak_header  pak_h ;
    int HEADER = sizeof(pak_h);

    //img_fifo /////////////////////////////////////////////
    int len  = IMG_WIDTH * IMG_HEIGHT * IMG_CHANNEL;

    int paklen = bLOCKUINT + HEADER + 64;
    unsigned char *buf_pak = new unsigned char[paklen*2];

    for(int j = 0;j< 2/*gDeviceIpAddList.cnt*/;j++)
    {
        gRecvImgfifo[j].NaldataFIFO_Init();
        if(gRecvImgfifo[j].error){
            printf("RecvImage_thread(): NaldataFIFO_Init() ERROR!");
            exit(1);
        }

        memset(buf_img[j],0,len);
    }

    int currentMode = pthis->m_modeImageSizeUp;
    CData passData;
    memset(&passData,0,sizeof(CData));

    int rlen = 0;
    char recvformip[64] = {0};
    int  recvformport = 0;

    pthis->gThPamss.running = 1;
    while(pthis->gThPamss.running == 1)
    {
        if(pthis->m_modeImageSizeUp  != currentMode)
        {
            currentMode = pthis->m_modeImageSizeUp;
            if(currentMode == 1)
                m_img_ch = 4;
            else if(currentMode == 2)
                m_img_ch = 1;
            else
                continue;

            len  = IMG_WIDTH * IMG_HEIGHT * m_img_ch;
            for(int j = 0;j< 2/*gDeviceIpAddList.cnt*/;j++)
            {
                gRecvImgfifo[j].NaldataFIFO_Release();
                gRecvImgfifo[j].NaldataFIFO_Init();
                if(gRecvImgfifo[j].error)
                {
                    printf("RecvImage_thread(): NaldataFIFO_Init() ERROR!");
                    exit(1);
                }
                ReSizeMemery(len,&buf_img[j],0);
            }

        }

        rlen = pthis->udpRecv(recvformip, &recvformport, (char*)buf_pak, paklen);
        if(rlen <= 36)
        {
            usleep(10); //ms: window->Sleep()
            continue;
        }

        //callback function
        memset(passData.remoteip, 0, sizeof(passData.remoteip));
        strcpy(passData.remoteip,recvformip);
        passData.recvLen = len;
        passData.remoteport = recvformport;
        passData.recv_pack = buf_pak;

        pthis->RegisterDataCallback(passData, pthis, OnNetDataCallbackFun);

        recvformport = 0;
        memset(recvformip, 0, sizeof(recvformip));

        //usleep(10 * 1000);
    }

    delete[] buf_pak ;
    for(int j = 0;j<gDeviceIpAddListsFunc.cnt;j++)
    {
        gRecvImgfifo[j].NaldataFIFO_Release();
    }

    pthis->gThPamss.begin     = 0;
    pthis->gThPamss.end       = 1;
    pthis->gThPamss.running   = 0;
    pthread_exit(NULL);

    return 0;
}

static void OnNetDataCallbackFun(CData pData, void* lparam)
{
    C_PMPAS* pthis = (C_PMPAS*)lparam;
    // packeg header
    pak_header  pak_h ;
    int HEADER = sizeof(pak_h);

    //img_fifo /////////////////////////////////////////////
    int len  = pData.recvLen;
    static int all_rlen[10] = {0};

    for(int i = 0;i< gDeviceIpAddListsFunc.cnt;i++)
    {     
        if(strncmp(pData.remoteip, gDeviceIpAddListsFunc.ScanIP_List[i], strlen(gDeviceIpAddListsFunc.ScanIP_List[i])) == 0)
        {
            //strcpy(gIpAdd,pData.remoteip);//////////////??????????????????globle varials is right here?
            memcpy(&pak_h.len, pData.recv_pack+16, 4);
            all_rlen[i] += pak_h.len ;

            memcpy(&pak_h.m, pData.recv_pack+32, 4);
            if(pak_h.m == 1)
            {
                if(all_rlen[i] != len)
                {
                    printf("lost frame number: %f \n", (float)(len - all_rlen[i])/bLOCKUINT);
                    fflush(stdout);

                    all_rlen[i] = 0;
                    usleep(10);
                    continue;
                }
            }
            memcpy(&pak_h.index, pData.recv_pack+20, 4);
            memcpy(&buf_img[i][pak_h.index], pData.recv_pack+HEADER, pak_h.len);

            if(all_rlen[i] == len)
            {
                g_index = i;
                //gRecvImgfifo[i].NaldataFIFO_PUSH(buf_img[i],all_rlen[i]);              
                //*pthis->m_cReady = true;
                pthis->doPassData(i, buf_img[i], all_rlen[i]);
                all_rlen[i] = 0;
            }
            else if(all_rlen[i] > len)
               all_rlen[i] = 0;
        }
    }
}

void* threadTcpSendCommand(void* lpParam)
{
    Embedded_Control* pthis = (Embedded_Control*)lpParam;
    if(pthis == NULL)
        exit(1);

    pthis->ThPam[EM_ENUM_ID_TCP_RECV].begin = 1;

    int ret = 0;
    Msg CmdMsg;
    CmdMsg.msgType = 0;
    struct msqid_ds ds_buf;

    pthis->ThPam[EM_ENUM_ID_TCP_RECV].running = 1;
    while(pthis->ThPam[EM_ENUM_ID_TCP_RECV].running == 1)
    {
        if(pthis->ctlNetTcpConnectStatus() >= 0)
        {
            memset(CmdMsg.msgText, 0x00, sizeof(CmdMsg.msgText));
            ret = msgrcv(pthis->m_msgid, (void*)&CmdMsg, MAX_MSG_LEN, CmdMsg.msgType ,1);
            if(ret < 0)
            {
                printf("msgrcv() error: %s\n", strerror(errno));
                break;
            }

            long msglength = sizeof(CmdMsg.msgText);
            int sendnum = pthis->ctlNetTcpSendMsg(CmdMsg.msgText,msglength);
            if(sendnum != msglength)
               printf(".Msg form CmdMsgQueue send failed! \n");

            if(strncmp(CmdMsg.msgText,GUI_CMD_CONTROL_EXIT,7) == 0)
                pthis->ThPam[EM_ENUM_ID_TCP_RECV].running  = 0;
        }
        else
        {
            printf("%s %d-->tcp disconnected. \n",__FILE__,__LINE__);
            break;
        }
        usleep(10 * 1000);    //1 second delay
    }

    ret = msgctl(pthis->m_msgid, IPC_RMID, &ds_buf);
    if(ret != 0)
    {
        printf("msgctl() error: %s\n", strerror(errno));
    }

    printf("msgctl() success. MQ is deleted\n");

    pthis->ThPam[EM_ENUM_ID_TCP_RECV].begin     = 0;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV].end       = 1;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV].running   = 0;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV].err_code  = 0;
    pthread_exit(NULL);

    return 0;
}

void *threadDecodegTcpMsg(void* lpParam)
{
    Embedded_Control* pthis = (Embedded_Control*)lpParam;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].begin = 1;

    // 接收buffer
    char *buf = new char [1024];
    int len = 1024;
    int recvbytes = 0;

    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running = 1;
    while(pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running == 1)
    {
        if(pthis->m_bConnectStatus)
        {
          if((recvbytes=pthis->ctlNetTcpRecvMsg(buf,len)) <= 0)
          {
              int tmp = pthis->ctlNetTcpConnectStatus();
              if(tmp == -1){
                  break;
              }
              else{
                  MyDelay(10);
                  continue;
              }
          }

          buf[recvbytes] = '\0';
          pthis->decodeTcpServerMsg(buf);
        }
        else
            MyDelay(10);
    }

    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].begin     = 0;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].end       = 1;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running   = 0;
    pthis->ThPam[EM_ENUM_ID_TCP_RECV_RETURE].err_code  = 0;
    pthread_exit(NULL);
    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Embedded_Control CLASS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Embedded_Control::initThreadParameter()
{
    for(int i=0; i < THREADCNT; i++)
    {
        ThPam[i].begin	  = 0 ;
        ThPam[i].running  = 0 ;
        ThPam[i].end	  = 0 ;
        ThPam[i].err_code = 0 ;
    }
}

void Embedded_Control::initDefineParameter()
{
    tcp = new TCPclass();
    tcp->LocalIP = NULL;
    RecvImgfifo = new ImageFIFO();
    nFrameID = 0;
    m_Ready = false;
    m_bDevScan = false;
    m_bConnectStatus = false;

    userCmdBuffer[0]  = {10,&userCmdParam.cmd_head};
    userCmdBuffer[1]  = {6 ,&userCmdParam.cmd_type};
    userCmdBuffer[2]  = {32,&userCmdParam.cmd_ip};
    userCmdBuffer[3]  = {8 ,&userCmdParam.cmd_port};
    userCmdBuffer[4]  = {1024,&userCmdParam.cmd_data};
    userCmdBuffer[5]  = {8 ,&userCmdParam.cmd_tril};

    for(int i = 0;i<CMDCNT;i++)
        memset((char*)userCmdBuffer[i].pdata, 0 ,sizeof(char)*userCmdBuffer[i].inum);

    mutex = PTHREAD_MUTEX_INITIALIZER;  //static
    cond = PTHREAD_COND_INITIALIZER;
}

void Embedded_Control::initMsgParameter(int MSQ_KEY)
{
    m_msgid = msgget((key_t)MSQ_KEY, 0660|IPC_CREAT|IPC_EXCL);
    if( m_msgid == -1)
    {
        if(errno == EEXIST)
        {
            /*Message queue has aleardy existed */
            printf("msgget() warning: %s\n", strerror(errno));
            m_msgid = msgget((key_t)MSQ_KEY, 0660|IPC_CREAT); /*access the mq*/
            if(m_msgid == -1)
            {
                printf("msgget() error: %s\n", strerror(errno));
                exit(1);
            }
        }
        else
        {
            /*msgget error*/
            printf("msgget() error: %s\n", strerror(errno));
            exit(1);
        }
    }
    printf("msgget() success. shmid=[%d]\n", m_msgid);
}

void Embedded_Control::decodeTcpServerMsg(char* ch_back)
{
    int temp = (unsigned char)ch_back[0] + 0;

    int writeByte = 0;
    unsigned char c_tempvalue[5]= {0};

    unsigned char c_sysinfo[255] = {0};
    unsigned char c_fileini[1024] = {0};

    switch(temp)
    {
    case 2: //runing time
        strncpy((char*)c_tempvalue,&ch_back[1],4);
        memcpy(&gpowerOnTime,c_tempvalue,4);
        break;
    case 3: //system information
        strncpy((char*)c_sysinfo,&ch_back[1],80);
        break;
    case 5: //get config ini file
        int fd;
        memcpy(&writeByte,&ch_back[1],sizeof(writeByte));
        strncpy((char*)c_fileini,&ch_back[1+sizeof(writeByte)],writeByte);
        strcat(iniPath,"config.ini");
        fd = open(iniPath, O_RDWR|O_CREAT, 0777);
        writeByte = write(fd, c_fileini, writeByte);
        close(fd);
    case 11:
        strncpy((char*)c_tempvalue,&ch_back[1],4);
        memcpy(&gallFrameCnt, c_tempvalue, 4);
        break;
    case 12:
        strncpy((char*)c_tempvalue,&ch_back[1],4);
        memcpy(&gframeRate, c_tempvalue, 4);
        break;
    case 13:
        strncpy((char*)c_tempvalue,&ch_back[1],4);
        memcpy(&ginSideTemp, c_tempvalue, 4);
        break;
    case 14:
        strncpy((char*)c_tempvalue,&ch_back[1],4);
        memcpy(&gtempStatu, c_tempvalue, 4);
        break;
    default:
        printf("%s",ch_back);
        break;
    }
    return;
}

int Embedded_Control::ctlNetTcpRecvMsg(char *buf, int len)
{
  return this->tcp->tcpRecvMsg(buf, len);
}

int Embedded_Control::ctlCmdDeviceStart()
{
    return this->ctlSetCmdStatuss(EM_ON);
}

int Embedded_Control::ctlCmdDeviceStop()
{
  return this->ctlSetCmdStatuss(EM_OFF);
}

int Embedded_Control::ctlCmdDeviceExit()
{
  return this->ctlSetCmdExit();
}

int Embedded_Control::ctlCmdDeviceSW2DMode()
{
  return this->ctlSetCmdSysMode(EM_2D_ORGLOOK);
}

int Embedded_Control::ctlCmdDeviceSW3DMode()
{
  return this->ctlSetCmdSysMode(EM_NORMAL_3D);
}

int Embedded_Control::ctlCmdDeviceSWIdleMode()
{
  return this->ctlSetCmdSysMode(EM_IDLE);
}

int Embedded_Control::ctlCmdDeviceProjectorOpen()
{
  return this->ctlSetCmdProjectorSW(EM_ON);
}

int Embedded_Control::ctlCmdDeviceProjectorClose()
{
  return this->ctlSetCmdProjectorSW(EM_OFF);
}

int Embedded_Control::ctlCmdDeviceCalcModeARM()
{
  return this->ctlSetCmdCalcMode(EM_ON);
}

int Embedded_Control::ctlCmdDeviceCalcModePC()
{
  return this->ctlSetCmdCalcMode(EM_OFF);
}

int Embedded_Control::ctlCmdDeviceRunOnceMode()
{
  return this->ctlSetCmdRun3DMode(EM_OFF);
}

int Embedded_Control::ctlCmdDeviceTrigSourceIn()
{
  return this->ctlSetCmdTriggerSource(EM_OFF);
}

int Embedded_Control::ctlCmdDeviceTrigSourceOut()
{
  return this->ctlSetCmdTriggerSource(EM_ON);
}

int Embedded_Control::ctlCmdDeviceRunMultiMode()
{
  return this->ctlSetCmdRun3DMode(EM_ON);
}

int Embedded_Control::ctlCmdDeviceLoadConfigini(const char *loadputfilepath)
{
    strcpy(iniPath, loadputfilepath);
    return this->ctlSetCmdLoadConfigini();
}

int Embedded_Control::ctlCmdDeviceRecoveConfigini()
{
  return this->ctlSetCmdRecovConfigini();
}

int Embedded_Control::ctlCmdDeviceSendConfigini(const char *sendfilepath)
{
    return this->ctlSetCmdSendConfigini(sendfilepath);
}

int Embedded_Control::ctlCmdDeviceGetSysStatus(uint64_t *powerOnTime, uint64_t *allFrameCnt, int32_t *frameRate, int32_t *inSideTemp, int32_t *tempStatu)
{
    int res = 0;
    res = this->ctlGetCmdSystemCurrentStatus();
    MyDelay(10);
    *powerOnTime = gpowerOnTime;
    *allFrameCnt = gallFrameCnt;
    *frameRate = gframeRate;
    *inSideTemp = ginSideTemp;
    *tempStatu = gtempStatu;
    return res;
}

int Embedded_Control::ctlCmdDeviceExposureTime(const uint64_t _exTime)
{
    return this->ctlSetCmdCameraExposureTime(_exTime);
}

//static PNP_FRAME_CALLBACK_PARAM *pFrame;
void* Embedded_Control::ctlRegisterEmCallback(void* pUserParam, CallBackFunc callBackFunc)
{
    this->ctlImageCallBackFun = callBackFunc;
    pthread_create( &ThPam[EM_ENUM_ID_CALLBACK].id,     // Thread identifiers
            NULL,                                       // default security attributes
            threadCallBackToUser,                    // thread function
            (void *)this			// argument to thread function
    );

    while(ThPam[EM_ENUM_ID_CALLBACK].running == 0)
    {
        usleep(100);
    }
    return (void*)pUserParam;
}

void* Embedded_Control::ctlUnregeisterEmCallback()
{
    ctlImageCallBackFun = NULL;
    ThPam[EM_ENUM_ID_CALLBACK].running = 0;
    while(ThPam[EM_ENUM_ID_CALLBACK].end != 1)
        usleep(10 * 1000);
    return 0;
}

void* Embedded_Control::ctlUserCallBackFunmutex()
{
    pthread_mutex_lock(&this->mutex);
    this->m_Ready = false;
    if(this->ctlImageCallBackFun != NULL)
    {
        this->tclFrameParam[0].nFrameID  = this->nFrameID;
        this->ctlImageCallBackFun(this->tclFrameParam);
    }
    pthread_mutex_unlock(&this->mutex);

    return 0;
}

int Embedded_Control::ctlInitTcpNetWorkParamLocal(DeviceIpAddListsFunc* DeviceIpAddLists,int ip_index)  //default -> 0
{
    char localips[10][64] = {0};
    char remoteip[10][64] = {0};
    int device_cnt = 0;

    if(!m_bDevScan) //if not commit find_device,then use the configure(default) ip address
    {
        bool local_configure_ok = Read_configure(localips[ip_index],remoteip[ip_index], &device_cnt);
        if(local_configure_ok)
        {
            if(ip_index <= device_cnt)
            {
                strncpy(DeviceIpAddLists->LocalIP_List[ip_index], localips[ip_index],strlen(localips[ip_index]));
                strncpy(DeviceIpAddLists->ScanIP_List[ip_index], remoteip[ip_index],strlen(remoteip[ip_index]));
                tcp->LocalIP = remoteip[ip_index];
            }
        }
        else
        {
            tcp->getLocalIPs(localips);
            strncpy(DeviceIpAddLists->LocalIP_List[ip_index], localips[ip_index],strlen(localips[ip_index]));
            strncpy(DeviceIpAddLists->ScanIP_List[ip_index], tcp->IP,strlen(tcp->IP));
            tcp->LocalIP = tcp->IP;
        }
    }
    else    //using find device address
    {
        //strcpy(localips[ip_index],DeviceIpAddLists->ScanIP_List[ip_index]);
        tcp->LocalIP = (char*)DeviceIpAddLists->ScanIP_List[ip_index];
    }

//    DeviceIpAddLists->cnt ++;
    this->tcp->Netclass::setCurrentIpAddr(this->tcp->LocalIP,initDefaultPortTcp);
    return 0;
}

int Embedded_Control::ctlNetTcpConnectStatus()
{
    return this->Opt_Net_Status();
}

int Embedded_Control::ctlNetTcpSendMsg(char* msgBuf, long msgLength)
{
    return tcp->tcpSendMsg(msgBuf,msgLength);
}

bool Embedded_Control::Read_configure(char* loaclip, char* remoteip, int* device_cnt)
{
    loaclip = remoteip;
    remoteip = loaclip;
    *device_cnt = 1;
    return false;
}

char* Embedded_Control::emNet_RemoteIp()
{
    return tcp->LocalIP;
}

int Embedded_Control::Opt_Connect_Tcp()
{
    int ret = 0;
    int connect_cnt = 0;
    if(m_bConnectStatus)
        return 0;
    else
    {
        int connect_flag = tcp->tcpConnect();
        if(connect_flag >= 0)
        {
            ret = 0;
            m_bConnectStatus = true;
        }
        else
        {
            do{
                connect_cnt++;
                connect_flag = tcp->tcpConnect();
                if(connect_flag >= 0)
                {
                    break;
                }
                usleep(10 * 1000);
            }while(connect_cnt < 5);

            if(connect_flag < 0)
            {
                ret = -1;
                m_bConnectStatus = false;
                tcp->udpERROR(connect_flag + 5);
            }
        }
        if(connect_flag >= 0)
        {
            pthread_create(&ThPam[EM_ENUM_ID_TCP_RECV].id,     // Thread identifiers
                    NULL,                                       // default security attributes
                    threadTcpSendCommand,                    // thread function
                    (void *)this			// argument to thread function
            );

            pthread_create(&ThPam[EM_ENUM_ID_TCP_RECV_RETURE].id,
                    NULL,
                    threadDecodegTcpMsg,
                    (void*)this
            );

            while(ThPam[EM_ENUM_ID_TCP_RECV].running == 0)
            {
                usleep(100);
            }

            while(ThPam[EM_ENUM_ID_TCP_RECV_RETURE].running == 0)
            {
                usleep(100);
            }
        }
    }
    return ret;
}

int Embedded_Control::ctlConnectTcp()
{
  return Opt_Connect_Tcp();
}

int Embedded_Control::Opt_DisConnect_Net(EM_NET_TYPE netype)
{
    int ret = 0;
    switch(netype)
    {
    case EM_NET_TCP:
        ret = tcp->tcpDisconnect();
        break;
    default:
        break;
    }
    return ret;
}

int Embedded_Control::Opt_Net_Status()
{
    return tcp->socketConnected();
}


int Embedded_Control::Opt_Net_Reset_Port(int port, EM_NET_TYPE netype)
{
    int ret = 0;
    switch(netype)
    {
    case EM_NET_TCP:
        tcp->setCurrentIpAddr(tcp->IP, port);
        break;
    default:
        break;
    }
    return ret;
}

int Embedded_Control::Opt_Cmd_Generata()
{
    Msg sendDate;
    sendDate.msgType = 1;
    memset(sendDate.msgText, 0 ,MAX_MSG_LEN);
    for(int i = 0;i<CMDCNT;i++)
    {
        strcat(sendDate.msgText,(char*)userCmdBuffer[i].pdata);
    }
    //int msglength = strlen(sendDate.msgText);
    //ctlNetTcpSendMsg(sendDate.msgText,msglength);
    if(msgsnd(m_msgid, (void*)&sendDate, MAX_MSG_LEN, IPC_NOWAIT) == -1)
    {
        return -1;
    }
    return 0;
}

int Embedded_Control::Opt_Cmd_Msg(char *msgtype, ...)
{
    userCmdBuffer[4].pdata = (char*)msgtype;
    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdStatuss(EM_STATU_SW isStart)
{
    if(isStart == EM_ON)
        userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_ACTIVE_START;
    else
        userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_ACTIVE_STOP;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;

    return 0;
}

int Embedded_Control::ctlSetCmdExit()
{
    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_EXIT;
    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdSysMode(EM_RUN_LEVEL sysMode)
{
    int ret = 0;
    switch(sysMode)
    {
    case EM_IDLE:
        userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_TO_IDLE_MODE;
        break;
    case EM_NORMAL_3D:
        userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_TO_NORMAL_MODE;
        break;
    case EM_2D_ORGLOOK:
        userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_TO_2DCAMERA_MODE;
        break;
    default:
        break;
    }

    if(this->Opt_Cmd_Generata() < 0)
        ret = -1;
    return ret;
}

int Embedded_Control::ctlSetCmdProjectorSW(EM_STATU_SW isOpen)
{
    if(isOpen == EM_ON)
        userCmdBuffer[4].pdata = (char *)GUI_CMD_PROJECTOR_ACTIVE_START;
    else
        userCmdBuffer[4].pdata = (char *)GUI_CMD_PROJECTOR_ACTIVE_STOP;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdCalcMode(EM_CALC_MODE calcMode)
{
    char tempCalcmd[64] = {0};
    strcpy(tempCalcmd,(char *)GUI_CMD_CONTROL_SET_CAL_OFFORON);
    if(calcMode == EM_OFF) //EM
    {
        sprintf(&tempCalcmd[strlen(tempCalcmd)], "%d", 0);
    }
    else if(calcMode == EM_ON)
    {
        sprintf(&tempCalcmd[strlen(tempCalcmd)], "%d", 1);
    }
    userCmdBuffer[4].pdata = tempCalcmd;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdTriggerSource(EM_STATU_SW triggerSource)
{
    char tempTriggerSource[64] = {0};
    strcpy(tempTriggerSource,(char *)GUI_CMD_CONTROL_SET_TRIGGER_SOURCE);
    if(triggerSource == EM_OFF) //EM
    {
        sprintf(&tempTriggerSource[strlen(tempTriggerSource)], "%d", 0);
    }
    else if(triggerSource == EM_ON)
    {
        sprintf(&tempTriggerSource[strlen(tempTriggerSource)], "%d", 1);
    }
    userCmdBuffer[4].pdata = tempTriggerSource;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdRun3DMode(EM_3D_RUN_TYPE runType)
{
    char tempRunModlecmd[64] = {0};
    strcpy(tempRunModlecmd,(char *)GUI_CMD_CONTROL_SET_CAL_TYPE);
    if(runType == EM_OFF) //EM
    {
        sprintf(&tempRunModlecmd[strlen(tempRunModlecmd)], "%d", 0);
    }
    else if(runType == EM_ON)
    {
        sprintf(&tempRunModlecmd[strlen(tempRunModlecmd)], "%d", 1);
    }
    userCmdBuffer[4].pdata = tempRunModlecmd;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::rootSetCmdProjectorUploadPatterns()
{
    userCmdBuffer[4].pdata = (char *)GUI_CMD_PROJECTOR_TO_UPLOAD_MODE;
    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdCameraExposureTime(const uint64_t expTime)
{
    char tempExpcmd[64] = {0};
    strcpy(tempExpcmd,(char *)GUI_CMD_CALIB_SET_EXPOSURETIME);

    sprintf(&tempExpcmd[strlen(tempExpcmd)], "%ld", expTime);
    userCmdBuffer[4].pdata = tempExpcmd;

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdLoadConfigini()
{
    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_ACTIVE_LOAD_CONF;
    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdRecovConfigini()
{
    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_ACTIVE_RECOV_CONF;
    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlSetCmdSendConfigini(const char *_filepath)
{
    int readByte = 0;
    int fs = open(_filepath,O_RDWR,S_IRWXU|S_IRGRP);
    if(fs < 0)
        return fs;

    char tempchar[64] = {0};
    char tempConfini[1024] = {0};
    strcpy(tempchar,(char *)GUI_CMD_CONTROL_ACTIVE_SYNC_CONF);
    sprintf(&tempchar[strlen(tempchar)], "%d", 5);
    //文件内容
    readByte = read(fs, tempConfini, 1024);
    if(readByte < 0)
        return readByte;
    close(fs);

    strcpy((char*)userCmdBuffer[4].pdata, tempchar);
    strncat((char*)userCmdBuffer[4].pdata,tempConfini,readByte);

    if(this->Opt_Cmd_Generata() < 0)
        return -1;
    return 0;
}

int Embedded_Control::ctlGetCmdSystemCurrentStatus()
{
    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_GET_RUNTIME;//time running
    if(this->Opt_Cmd_Generata() < 0)
        return -1;

    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_GET_FPGATIME;//time calculate
    if(this->Opt_Cmd_Generata() < 0)
        return -1;

    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_GET_TEMPRATURE;//temprature
    if(this->Opt_Cmd_Generata() < 0)
        return -1;

    userCmdBuffer[4].pdata = (char *)GUI_CMD_CONTROL_GET_FRAMERATE;//framerate
    if(this->Opt_Cmd_Generata() < 0)
        return -1;

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UDP CLASS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
UDPclass::UDPclass()
{
    fd_udp = 0;
    memset(fromIP, 0, sizeof(fromIP));
    memset(toIP, 0, sizeof(toIP));
    
    bzero((char *)(&serv_udp), SocketAddressSize);
    bzero((char *)(&temp_udp), SocketAddressSize);
}

int UDPclass::createudpSocket()
{
    int res =0;
    //用UDP初始化套接字
    this->fd_udp = socket(AF_INET,SOCK_DGRAM,0);
    if(!(this->fd_udp))
    {
        return -1;
    }
    //时限
#if defined(_WIN32)
    int nNetTimeout=0.5*1000;//m秒
    res |= setsockopt(this->fd_udp,SOL_SOCKET,SO_SNDTIMEO,(char *)&nNetTimeout,sizeof(int));
    res |= setsockopt(this->fd_udp,SOL_SOCKET,SO_RCVTIMEO,(char *)&nNetTimeout,sizeof(int));
#else
    struct timeval nNetTimeout={0,500};
    res |= setsockopt(this->fd_udp,SOL_SOCKET,SO_SNDTIMEO,&nNetTimeout,sizeof(nNetTimeout));
    res |= setsockopt(this->fd_udp,SOL_SOCKET,SO_RCVTIMEO,&nNetTimeout,sizeof(nNetTimeout));
#endif
    if(res<0){ return -3; }

    // 缓冲区
    int nBuf=36*1024*1024;//设置为xM
    res|= setsockopt(this->fd_udp, SOL_SOCKET,SO_SNDBUF,(const char *)&nBuf,sizeof(int));
    res|= setsockopt(this->fd_udp, SOL_SOCKET,SO_RCVBUF,(const char *)&nBuf,sizeof(int));
    if(res < 0){ return -4; }

    return res;
}

int UDPclass::bindudp(char *bindIP)
{
    int res =0;

    this->serv_udp.sin_family		=	AF_INET			;
    this->serv_udp.sin_addr.s_addr	=	inet_addr(bindIP)	;	//INADDR_ANY ;
    this->serv_udp.sin_port			=	htons(PORT)	;
    // 把该套接字绑定在一个具体的地址上  !!!!!!! 注意这里 !!!!!!!
    // 这是与SOAP框架不同的地方，也是之所以可以跨网段的原因！
    res = bind(this->fd_udp,(sockaddr *)&serv_udp,SocketAddressSize);
    if(res < 0){
        return -2;
    }
    return res;
}

int UDPclass::bindudp(char *bindIP, int PORT)
{
    int res =0;

    this->serv_udp.sin_family		=	AF_INET			;
    this->serv_udp.sin_addr.s_addr	=	INADDR_ANY;
    this->serv_udp.sin_port			=	htons(PORT)	;
    // 把该套接字绑定在一个具体的地址上  !!!!!!! 注意这里 !!!!!!!
    // 这是与SOAP框架不同的地方，也是之所以可以跨网段的原因！
    res = bind(this->fd_udp,(sockaddr *)&serv_udp,SocketAddressSize);
    if(res < 0){
        printf("ip --> %s \n",bindIP);
        return -2;
    }
    return res;
}

int UDPclass::udpConnect()
{
    int res = createudpSocket();
    if(res < 0)
        return res;
    res = bindudp(LocalIP);
    if(res < 0)
        return res;

    return 0 ;
}

int UDPclass::setMultiBrocast(char *localip, char *muip)
{
    ip_mreq  mreq_recv ;

    mreq_recv.imr_interface.s_addr = inet_addr(localip)    ;//INADDR_ANY
    mreq_recv.imr_multiaddr.s_addr = inet_addr(muip)	;

    //加入组播
    int err = 0 ;
    err = setsockopt(fd_udp, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         &mreq_recv, sizeof(ip_mreq) );

    //关闭回显
    int optval = 0 ;
    err = setsockopt(fd_udp, IPPROTO_IP, IP_MULTICAST_LOOP,
                         &optval, sizeof(optval) );

    //时限
#if defined(_WIN32)
    int nNetTimeout=0.1*1000;//m秒
    err |= setsockopt(fd_udp,SOL_SOCKET,SO_SNDTIMEO,(char *)&nNetTimeout,sizeof(int));
    err |= setsockopt(fd_udp,SOL_SOCKET,SO_RCVTIMEO,(char *)&nNetTimeout,sizeof(int));
#else
    struct timeval nNetTimeout={0,100}; //Seconds,Microseconds
    err |= setsockopt(fd_udp,SOL_SOCKET,SO_SNDTIMEO,&nNetTimeout,sizeof(nNetTimeout));
    err |= setsockopt(fd_udp,SOL_SOCKET,SO_RCVTIMEO,&nNetTimeout,sizeof(nNetTimeout));
#endif

    return err ;
}

int UDPclass::tempSendudp(char *buf, int len, char *toIP, int toPort)
{
    int tolen = sizeof(struct sockaddr_in);

    this->temp_udp.sin_family		=	AF_INET			;
    this->temp_udp.sin_addr.s_addr	=	inet_addr(toIP)	;	//INADDR_ANY ;
    this->temp_udp.sin_port			=	htons(toPort)	;

    int ret =sendto(this->fd_udp, buf, len, 0, (const sockaddr *)&temp_udp, tolen);
//    perror("perror");
    return ret ;
}

int UDPclass::tempRecvudp(char *buf, int len, char *fromIP, int* fromPort)
{
    static int res = 0;
    socklen_t fromlength = sizeof(struct sockaddr_in);

    memset(buf, 0, len);
    for(int i = 0;i<5;i++)
    {
        res = recvfrom(this->fd_udp, buf, len, 0, (struct sockaddr *)&temp_udp, &fromlength);
        if(res < 0)
            continue;
        else
            break;
    }

    if(res > 0)
    {
        strcpy(fromIP, inet_ntoa(this->temp_udp.sin_addr));
        *fromPort = ntohl(this->temp_udp.sin_port);
    }
    return res ;
}

int UDPclass::udpSendMsg(char *buf, int len)
{
    return sendMsg(fd_udp,buf,len);
}

int UDPclass::udpRecvMsg(char *buf, int len)
{
    return recvMsg(fd_udp,buf,len);
}

int UDPclass::udpDisconnect()
{
    return disconnect(fd_udp);
}

/*********************
 * C_PMPAS
 * *****************/
C_PMPAS::C_PMPAS()
{
    udp_find = new UDPclass();
    udp_recvClint = new UDPclass();
    localIP_num = 0;
    remoteIP_num = 0;
    m_modeImageSizeUp = 0;

    for(int i = 0;i<32;i++)
        gui_DeviceScanInfo[i] = &gDeviceInfoScan[i];

    for(int i = 0;i<32;i++)
    {
        memset(localprobe_[i].probe_, 0, sizeof(localprobe_[i].probe_));
        localprobe_[i].localip_num = 0;
    }
    memset(udpLocalips, 0 , sizeof(udpLocalips));
    m_bDevScan = false;
    m_bDevOpen = false;

    memset (gDeviceIpAddListsFunc.used_ok, 0, sizeof(gDeviceIpAddListsFunc.used_ok));
    gDeviceIpAddListsFunc.cnt = 0;
    ///////////////////////////////////////////////////////////////////////////////////
    gThPamss.begin        = 0 ;
    gThPamss.running      = 0 ;
    gThPamss.end          = 0 ;
    gThPamss.err_code     = 0 ;

    for(int i = 0;i < 10;i++)
    {
        gRecvImgfifo[i].width = IMG_WIDTH;
        gRecvImgfifo[i].height = IMG_HEIGHT;
        gRecvImgfifo[i].channel = IMG_CHANNEL;
    }
    memset(gIpAdd,0,sizeof(gIpAdd));

    // 初始化互斥锁
    if (pthread_mutex_init(&thread_mutex, NULL) != 0){
       // 互斥锁初始化失败
       exit(EXIT_FAILURE);
    }

    for(int i = 0;i<10;i++)
      buf_img[i] = new unsigned char [IMG_WIDTH*IMG_HEIGHT*IMG_CHANNEL*sizeof(char)];

    m_bIsConnect = false;
    m_cReady = NULL;
}

C_PMPAS::~C_PMPAS()
{
    pthread_mutex_destroy(&thread_mutex);   //dynamics

//    gThPamss.running = 0;
//    while(gThPamss.end != 1)
//    {
//        gThPamss.running = 0;
//        usleep(10 * 1000);
//    }
}

int C_PMPAS::emInitudp(DeviceInfoScan* dev_list, int dev_cnt)
{
    int ret = 0;

//    if(dev_cnt < 1)
//        return -1;

    ret |= udpNetWorkParam(dev_list[dev_cnt].lIP);
    ret |= udpConnect();

    return ret;
}

int C_PMPAS::udpNetWorkParam(char* localip)  //default -> 0
{
    udp_recvClint->LocalIP = localip;
    udp_recvClint->Netclass::setCurrentIpAddr(udp_recvClint->LocalIP,initDefaultPortUdp);

    return 0;
}

int C_PMPAS::udpConnect()
{
    int ret = 0;
    int connect_cnt = 0;
    if(m_bIsConnect)
        return 0;
    else
    {
        int connect_flag = udp_recvClint->udpConnect();
        if(connect_flag >= 0)
        {
            ret = 0;
            m_bIsConnect = true;
        }
        else
        {
            do{
                connect_cnt ++;
                connect_flag = udp_recvClint->udpConnect();
                if(connect_flag >= 0)
                {
                    break;
                }
                usleep(10 * 1000);
            }while(connect_cnt < 5);

            if(connect_flag < 0)
            {
                m_bIsConnect = false;
                ret = -1;
                udp_recvClint->udpERROR(connect_flag + 5);
            }
        }
        if(connect_flag >= 0)
        {
            pthread_create(&this->gThPamss.id,     // Thread identifiers
                    NULL,                     // default security attributes
                    threadDecodegUdpUser,  // thread function
                    (void *)this			  // argument to thread function
            );
            while(gThPamss.running == 0)
            {
                usleep(100);
            }
        }
    }
    return ret;
}

int C_PMPAS::udpRecv(char *fromIP, int* fromPort, char *buf, int len)
{
    return udp_recvClint->tempRecvudp(buf, len, fromIP, fromPort);
}

void C_PMPAS::emScanDevice()
{
    int device_num = 0;
    for(int i = 0;i<32;i++)
        memset((char*)&gDeviceInfoScan[i], 0, sizeof(DeviceInfoScan));
    udpScanDevice((char*)MULTIBROADCAST_IP, MULTIBROADCAST_PORT);
    getDevTable(gDeviceInfoScan);

    for(int i = 0;i<32;i++)
    {
        if(strlen(gDeviceInfoScan[i].lIP) > 0)
            device_num++;
    }
    if(device_num > 0)
        m_bDevScan = true;
    else
        m_bDevScan = false;
}

void C_PMPAS::emAddIpList(char* ip, int ip_index)
{
    static bool test = false;
    if(!gDeviceIpAddListsFunc.used_ok[ip_index])
    {
        gDeviceIpAddListsFunc.used_ok[ip_index] = true;
        devPMP_Collect((void*)NULL, ip_index, 0);

        strcpy(gDeviceIpAddListsFunc.LocalIP_List[ip_index], gDeviceInfoScan[ip_index].lIP);
        strcpy(gDeviceIpAddListsFunc.ScanIP_List[ip_index], ip);
        gDeviceIpAddListsFunc.cnt = ip_index +1;

        brodcastRemoteIp(gDeviceIpAddListsFunc.LocalIP_List[ip_index], gDeviceIpAddListsFunc.ScanIP_List[ip_index], gDeviceInfoScan[ip_index].MAC);
        usleep(100*1000);
        //connect and create udp thread
        if(!test)
        {
            emInitudp(gDeviceInfoScan,ip_index);
            test = true;
        }
        else //only connect
        {
            udp_recvClint->LocalIP = gDeviceIpAddListsFunc.LocalIP_List[ip_index];
            udp_recvClint->udpConnect();
        }
    }
    else
    {
        printf("Current Operate IP: (Host) %s (Client)%s \n", gDeviceIpAddListsFunc.LocalIP_List[ip_index], gDeviceIpAddListsFunc.ScanIP_List[ip_index]);
        fflush(stdout);
    }
    ///////////do somethings//////////////////
}

int C_PMPAS::setMultiBrocast(char *muip, char *localip)
{
    return udp_find->setMultiBrocast(localip, muip);
}

int C_PMPAS::getDevTable(pDeviceInfoScan probee)
{
    int ret = 0;
    int temp_num = 0;
    for (int i=0; i < localIP_num; i++)
    {
        temp_num = localprobe_[i].localip_num;
        if(temp_num == 0)
            continue;
        for(int j = 0;j<temp_num;j++)
        {
            strcpy(probee->IP, localprobe_[i].probe_[j].IP);

            int a1,b1,c1,d1;
            sscanf(probee->IP,"%d.%d.%d.%d",&a1,&b1,&c1,&d1);

            int a,b,c,d;
            sscanf(udpLocalips[i],"%d.%d.%d.%d",&a,&b,&c,&d);

            if( (a == a1) && (b == b1) && (c == c1))
            {
                strcpy(probee->MAC, localprobe_[i].probe_[j].MAC);
                strcpy(probee->Name, localprobe_[i].probe_[j].Name);
                strcpy(probee->SN, localprobe_[i].probe_[j].SN);
                strcpy(probee->Version, localprobe_[i].probe_[j].Version);
                strcpy(probee->lIP, udpLocalips[i]);
                remoteIP_num ++;
            }
            ret++;
            probee++;//point to next DeviceInfo
        }
    }

    return ret;
}

char* C_PMPAS::udpXmltok(char *inbuf, const char *begin, const char *end, char *outbuf)
{
    //find begin/end positions
    char *k_b = strstr(inbuf,begin);
    char *k_e = strstr(inbuf,end);

    if(k_b == NULL || k_e == NULL){
        return NULL;
    }
    //no conects
    int len_b = strlen(begin);
    if(k_b+len_b == k_e)
        return NULL;

    //point to keywork-head position
    char *k = k_b + len_b ;

    int outlen =k_e - k_b - len_b;
    strncpy(outbuf, k, outlen);

    return k;
}

int C_PMPAS::udpDeodeXML(char *inxml, int Localip_index, int resv_d)
{
    char  tmp_arr[256];
    int ret = 0;
    if(resv_d < localprobe_[Localip_index].localip_num)
        resv_d = localprobe_[Localip_index].localip_num;

    UDP_PROB *Probe = &localprobe_[Localip_index].probe_[resv_d];
    localprobe_[Localip_index].localip_index = Localip_index;


    while(NULL != strstr(inxml,"<SHS-ENV:Header>"))
    {
        localprobe_[Localip_index].localip_num ++;
        //init variable values
        memset(Probe->IP		,'\0',32);
        memset(Probe->MAC		,'\0',32);
        memset(Probe->Types		,'\0',32);
        memset(Probe->Name		,'\0',64);
        memset(Probe->SN		,'\0',64);
        memset(Probe->Version	,'\0',64);

//        int in_len = strlen(inxml);
        //Decode wsa:To
        const char *wsaTo_key = "<wsa:To>" ;

        char *wsaTo_p = strstr(inxml,wsaTo_key);
        if( wsaTo_p != NULL ){	//ok
            wsaTo_p =strstr(wsaTo_p,"PC");
        }

        //Body
        const char *Body_key = "<Body:/discovery/Probe>" ;

        char *Body = strstr(inxml,Body_key);
        if( Body!=NULL && wsaTo_p!=NULL){
            char *rec =NULL;
            rec = udpXmltok(Body, "<wsdd:IP>", "</wsdd:IP>",Probe->IP);
            rec = udpXmltok(Body,"<wsdd:PORT>","</wsdd:PORT>",tmp_arr);
            if( rec != NULL ){
                sscanf(tmp_arr,"Video:%d/CMD:%d",
                    &Probe->Video_Port, &Probe->CMD_Port);
            }
            rec = udpXmltok(Body,"<wsdd:MAC>","</wsdd:MAC>",Probe->MAC);
            rec = udpXmltok(Body,"<wsdd:Types>","</wsdd:Types>",Probe->Types);
            rec = udpXmltok(Body,"<wsdd:Name>","</wsdd:Name>",Probe->Name);
            rec = udpXmltok(Body,"<wsdd:SN>","</wsdd:SN>",Probe->SN);
            rec = udpXmltok(Body,"<wsdd:Version>","</wsdd:Version>",Probe->Version);
        }
        //end Body
        inxml = strstr(inxml,"</Body:/discovery/Probe>");
        if( inxml == NULL ){
            continue;
        }
        //end Header
        inxml = strstr(inxml,"</SHS-ENV:Header>");
        if( inxml == NULL ){
            continue;
        }
        ret ++;
        //Probe++;  //maybe also works
        Probe = &localprobe_[Localip_index].probe_[ret];

    }//end for
    return ret;
}

int C_PMPAS::udpEncodeXML(const char *cxml, char *nxml)
{
    //////////////////////////////////////////////////
    // windows:GUID , Linux:UUID
    char MessageID[128] = {0};
    uuid_t uuid;
    uuid_generate(uuid);
    if(uuid_is_null(uuid) != 1) //If the value is equal to the NULL UUID, 1 is returned, otherwise 0 is returned.
    {
        uuid_unparse(uuid, MessageID);
    }
    else
    {
        strcpy(MessageID,"a5e4fffc-ebb3-4e9e-913e-7eecdf0b05e8");
    }

    int xml_len = strlen(cxml);
    char *xml = new char[xml_len+2] ;	//ŽÓconst char µœ char
    memset(xml, 0, xml_len+2);
    memcpy(xml, cxml, xml_len);

    char *tmp_xml = strstr(xml,"urn:uuid:");
    if(tmp_xml != NULL)
        tmp_xml += strlen("urn:uuid:");
    //MessageID
    int idlen = strlen(MessageID);
    for(int i=0;i<idlen;i++)
    {
        tmp_xml[i] = MessageID[i] ;
    }

    memcpy(nxml, xml, xml_len);
    delete xml;
    return 0;

}

int C_PMPAS::udpScanDevice(char * MultiBrocastIP, int MultiBrocastPort)
{
    if( MultiBrocastIP == NULL || MultiBrocastPort < 1 )
    {
         printf("input param error !! \r\n");
         return -1;
    }
    if( MultiBrocastIP[0] == '\0' || MultiBrocastIP[0] == 0 )
    {
         printf("input param error !! \r\n");
         return -1;
    }

    memset(udpLocalips, 0, sizeof(udpLocalips));
    for(int i = 0;i<32;i++)
    {
        memset(localprobe_[i].probe_, 0, sizeof(localprobe_[i].probe_));
        localprobe_[i].localip_num = 0;
        localprobe_[i].localip_index = 0;
    }
    /////////////////////////////////////////////////////////////
    // init
    udp_find->PORT		= MultiBrocastPort ;
    udp_find->LocalIP                               = NULL  ;
    memset(udp_find->fromIP	,'\0',64);
    memset(udp_find->toIP	,'\0',64);
    strcpy(udp_find->fromIP	,MultiBrocastIP); //239.255.255.250
    strcpy(udp_find->toIP	,MultiBrocastIP);

    const char  *cxml = {
        "<SHS-ENV:Header>"
        "<wsa:MessageID>urn:uuid:9D7A37A4-DBFE-4bdd-A79C-74998B7A375D</wsa:MessageID>"
        "<wsa:Action>urn:shs-xmlpmp3d:2019:05:/discovery/Probe</wsa:Action>"
        "<wsa:To>embedded_pmp3d</wsa:To>"
        "<Body:/discovery/Probe>"
        "<wsdd:IP>xxx.xxx.xxx.xxx</wsdd:IP>"
        "<wsdd:PORT>Video:0000/CMD:0000</wsdd:PORT>"
        "<wsdd:MAC>xx:xx:xx:xx:xx:xx</wsdd:MAC>"
        "<wsdd:Types>123456789</wsdd:Types>"
        "<wsdd:Name>test</wsdd:Name>"
        "<wsdd:SN>12345678</wsdd:SN>"
        "<wsdd:Version>0.0.0</wsdd:Version>"
        "<wsdd:Restor>19860801</wsdd:Restor>"
        "</Body:/discovery/Probe>"
        "</SHS-ENV:Header>"
    };
    char* xml = (char*)malloc(sizeof(char)*strlen(cxml)*1.5);
    udpEncodeXML(cxml,xml);
    
    int xml_len = strlen(xml);
    
    /////////////////////////////////////////////////////////////////
    // -- SET UDP
    int res = udp_find->createudpSocket();
    if(res)
    {
        printf("UDP.creat_fd() ERROR %d\n",res);
        udp_find->udpDisconnect();
        return -1 ;
    }
    localIP_num = udp_find->getLocalIPs(udpLocalips);
    if( localIP_num <= 0 )
    {
        printf("UDP.GetLocalIPs() ERROR !! IP num = %d \n",localIP_num);
        udp_find->udpDisconnect();
        return -1 ;
    }

    int get_num = 0;
    for(int index=0; index<localIP_num; index++)
    {
        udp_find->udpDisconnect();
        udp_find->createudpSocket();
        //strncpy(LocalIP, udpLocalips[index],strlen(udpLocalips[index]));
        udp_find->LocalIP = udpLocalips[index];
        int res = udp_find->bindudp(udp_find->LocalIP, udp_find->PORT);
        if (res) {
            printf("UDP %s .bindu() ERROR %d\n", udp_find->LocalIP,res);
            continue;
        }
        res = setMultiBrocast((char*)MULTIBROADCAST_IP,udp_find->LocalIP);
        if( res ){
            printf("UDP %s .SetMultiBrocast() ERROR %d\n", udp_find->LocalIP, res);
            continue;
        }

        //send Message
        udp_find->tempSendudp(xml, xml_len, udp_find->toIP, udp_find->PORT);
        /////////////////////////////////////////////////////////////////
        //wait for replay, decode message ,at most can resave 128 packages
        int num = 128 ;

        for(int n=0; n<num; n++)
        {
            char buf[4096] ;
            int size = 4096 ;
            res = 0 ;
            //resave 10 times (10*0.5),if no message,exit
            for(int j=0; j < 10; j++)
            {
                res += udp_find->tempRecvudp(buf+res, size-res, udp_find->fromIP, &udp_find->PORT);
                if( res >= (int)sizeof(cxml)/2 )
                {
                    break; //for j
                }
                else if(res < 0)
                    res = 0;
            }
            if(res == 0)
            {
                break;	//timeout
            }
            printf("recv:%d %d\n",n,res);
            fflush(stdout);
            //decode the recv buffer
            res = this->udpDeodeXML(buf, index, n);
            if( res >= 0 )
            {
                get_num += res ;	//the xml get number
            }

            if(get_num >= num)
            {
                break;
            }
        }
    }
    udp_find->udpDisconnect();
    free(xml);
    return get_num;
}

void C_PMPAS::brodcastRemoteIp(char* localip, char* remoteip, char* remotemac)
{
    if(localip == NULL || remoteip == NULL)
        exit(1);

    const char  *cxml = {
        "<SHS-ENV:Header>"
        "<wsa:MessageID>urn:uuid:9D7A37A4-DBFE-4bdd-A79C-74998B7A375D</wsa:MessageID>"
        "<wsa:Action>urn:shs-xmlpmp3d:2019:05:/discovery/SetIp</wsa:Action>"
        "<wsa:To>embedded_pmp3d</wsa:To>"
        "<Body:/discovery/SetIp>"
        "<wsdd:IP>xxx.xxx.xxx.xxx</wsdd:IP>"
        "<wsdd:MAC>xx:xx:xx:xx:xx:xx</wsdd:MAC>"
        "</Body:/discovery/SetIp>"
        "</SHS-ENV:Header>"
    };

    char* xml = (char*)malloc(sizeof(char)*strlen(cxml)*1.5);
    udpEncodeXML(cxml,xml);
    setDiscoveryXML(xml, xml, remoteip, remotemac);
    int xml_len = strlen(xml);

    //udp_find->udpDisconnect();
    udp_find->createudpSocket();
    //strncpy(LocalIP, udpLocalips[index],strlen(udpLocalips[index]));
    udp_find->LocalIP = localip;
    udp_find->PORT	  = MULTIBROADCAST_PORT;
    int res = udp_find->bindudp(udp_find->LocalIP, udp_find->PORT);
    if (res) {
        printf("UDP %s .bindu() ERROR %d\n", udp_find->LocalIP,res);
        exit(1);
    }
    res = setMultiBrocast((char*)MULTIBROADCAST_IP,udp_find->LocalIP);
    if( res ){
        printf("UDP %s .SetMultiBrocast() ERROR %d\n", udp_find->LocalIP, res);
        exit(1);
    }
    //send Message
    udp_find->tempSendudp(xml, xml_len, udp_find->toIP, udp_find->PORT);
    usleep(10 * 1000);
    udp_find->udpDisconnect();

}

int C_PMPAS::setDiscoveryXML(char *outxml, const char *inxml, char* setremoteip, char* setremotemac)
{
    char 	IP[128] 		;	memset(IP		,'\0',128);
    char 	MAC[128]		;	memset(MAC		,'\0',128);
    char 	wsaTo[128]		;	memset(wsaTo	,'\0',128);

    int in_len = strlen(inxml);

    memcpy(outxml, inxml, in_len);

    //Body
    const char *Body_key = "<Body:/discovery/SetIp>" ;

    char *Body = strstr(outxml,Body_key);
    if( Body!=NULL ){
        if( strstr(Body,"<wsdd:IP>")!=NULL ){
            sprintf(IP,"<wsdd:IP>%s</wsdd:IP>",setremoteip);
        }
        if( strstr(Body,"<wsdd:MAC>")!=NULL ){
            sprintf(MAC,"<wsdd:MAC>%s</wsdd:MAC>",setremotemac);
        }
    }//end if Body

    //wsa:To
    const char *wsaTo_key = "<wsa:To>" ;

    char *wsaTo_p = strstr(outxml,wsaTo_key);
    if( wsaTo_p!=NULL ){
        sprintf(wsaTo,"<wsa:To>embedded_pmp3d</wsa:To>");
    }

    sprintf(wsaTo_p, "%s"
                     "<Body:/discovery/SetIp>"
                     "%s%s"
                     "</Body:/discovery/SetIp>"
                     "</SHS-ENV:Header>",
                     wsaTo,IP,MAC);

    return 0 ;
}
///
/// \brief C_PMPAS::emOpenDevice
/// \param hDevice
/// \param nDeviceIndex must be Consistented with the order of the list obtained
/// \param msg_id
///
void C_PMPAS::emOpenDevice(EM_DEV_HANDLE* hDevice, int nDeviceIndex, int msg_id)
{
    *hDevice = new Embedded_Control(msg_id);
    //Embedded_Control* pthis = (Embedded_Control*)hDevice;
    m_bDevOpen = true;
//    if(nDeviceIndex >= 0 && nDeviceIndex < gDeviceIpAddListsFunc.cnt)
//    {
//        devPMP_Collect(hDevice,nDeviceIndex, 1);
//        if(m_bDevScan)
//            pthis->m_bDevScan = true;
//        else
//            pthis->m_bDevScan = false;

//        pthis->ctlInitTcpNetWorkParamLocal(&gDeviceIpAddListsFunc, nDeviceIndex);
//        int ret = pthis->ctlConnectTcp();
//        if(ret < 0)
//            printf("[%p] connect net error...\n", pthis);
//    }
//    pthis = NULL;
}

void C_PMPAS::emCloseDevice(EM_DEV_HANDLE* hDevice)
{
    if(*hDevice == NULL)
        return;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    m_bDevOpen = false;

    pthis->ctlRelease(* hDevice);
    delete *hDevice;
    *hDevice = NULL;
}

void C_PMPAS::emClientNetParam(EM_DEV_HANDLE hDevice, int nDeviceIndex)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;

    devPMP_Collect(hDevice,nDeviceIndex,1);
    if(m_bDevScan)
        pthis->m_bDevScan = true;
    else
        pthis->m_bDevScan = false;

    pthis->ctlInitTcpNetWorkParamLocal(&gDeviceIpAddListsFunc, nDeviceIndex);
    int ret = pthis->ctlConnectTcp();
    if(ret < 0)
        printf("[%p] connect net error...\n", pthis);

    /////////////////////////////////////////
}

//this maybe no work as think or we want
void* C_PMPAS::emRegisterImageCallback(EM_DEV_HANDLE hDevice, void *pUserParam, CallBackFunc callBackFunc)
{
    pHANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    m_cReady = &pthis->m_Ready;
    return (void*) pthis->ctlRegisterEmCallback(pUserParam,callBackFunc);
}

int C_PMPAS::emSetRunMode(EM_DEV_HANDLE hDevice, EM_RUN_MODE runmode, int outputType, int outputMode, int triggerType=0/*inside trigger*/)
{
    int emRes = 0;
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(runmode == EM_RUN_2D_MODE)
    {
        m_modeImageSizeUp = 2;
        emRes = pthis->ctlCmdDeviceSW2DMode();
    }
    else if(runmode == EM_RUN_3D_MODE)
    {
        m_modeImageSizeUp = 1;
        emRes = pthis->ctlCmdDeviceSW3DMode();
    }

    if(outputType == 0)
        emRes = pthis->ctlCmdDeviceCalcModePC();
    else
        emRes = pthis->ctlCmdDeviceCalcModeARM();

    if(outputMode == 0)
        emRes = pthis->ctlCmdDeviceRunOnceMode();
    else
        emRes = pthis->ctlCmdDeviceRunMultiMode();

    if(triggerType == 0)
        emRes = pthis->ctlCmdDeviceTrigSourceIn();
    else
        emRes = pthis->ctlCmdDeviceTrigSourceOut();
    return emRes;
}

int C_PMPAS::emSetExposureTime(EM_DEV_HANDLE hDevice, int extime)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    pthis->ctlCmdDeviceExposureTime(extime);
    return 0;
}
///
/// \brief C_PMPAS::emSetOutputResultsType
/// \param hDevice
/// \param outputType 0:disp 1:pointCloud
/// \return
///
int C_PMPAS::emSetOutputResultsType(EM_DEV_HANDLE hDevice, int outputType)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(outputType == 0)
        pthis->ctlCmdDeviceCalcModePC();
    else
        pthis->ctlCmdDeviceCalcModeARM();
    return 0;
}
///
/// \brief C_PMPAS::emSetTriggerSource
/// \param hDevice
/// \param triggerSource 0:inside trigger 1:outside trigger
/// \return
///
int C_PMPAS::emSetTriggerSource(EM_DEV_HANDLE hDevice, int triggerSource)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(triggerSource == 0)
        pthis->ctlCmdDeviceTrigSourceIn();
    else
        pthis->ctlCmdDeviceTrigSourceOut();
    return 0;
}
///
/// \brief C_PMPAS::emSetOutputResultsMode
/// \param hDevice
/// \param outputMode 0:singal 1:continue
/// \return
///
int C_PMPAS::emSetOutputResultsMode(EM_DEV_HANDLE hDevice, int outputMode)
{
    int emRes = 0;
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(outputMode == 0)
        emRes = pthis->ctlCmdDeviceRunOnceMode();
    else
        emRes = pthis->ctlCmdDeviceRunMultiMode();
    return emRes;
}

int C_PMPAS::emSwitchRunMode(EM_DEV_HANDLE hDevice, int sw_mode)
{
    int emRes = 0;
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(sw_mode == EM_2D_ORGLOOK)
    {
        m_modeImageSizeUp = 2;
        emRes = pthis->ctlCmdDeviceSW2DMode();
    }
    else if(sw_mode == EM_NORMAL_3D)
    {
        m_modeImageSizeUp = 1;
        emRes = pthis->ctlCmdDeviceSW3DMode();
    }
    else if(sw_mode == EM_IDLE)
    {
        m_modeImageSizeUp = 0;
        emRes = pthis->ctlCmdDeviceSWIdleMode();
    }
    return emRes;
}

void C_PMPAS::RegisterDataCallback(CData param, void* ppthis, void(*pNetDataRecv)(CData pData, void* lparam))
{
    pNetDataRecv(param, ppthis);
}

int C_PMPAS::emGetSysStatus(EM_DEV_HANDLE hDevice, uint64_t *allFrameCnt, uint64_t *powerOnTime, int32_t *frameRate, int32_t *inSideTemp, int32_t *tempStatu)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceGetSysStatus(powerOnTime, allFrameCnt, frameRate, inSideTemp, tempStatu);
}

int C_PMPAS::emGetSysSensorInfo(EM_DEV_HANDLE hDevice, int nDeviceIndex, char* ip, char *mac, char *sn, char *version, char *name)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    if(m_bDevScan)
    {
        strcpy(ip, gDeviceInfoScan[nDeviceIndex].IP);
        strcpy(mac, gDeviceInfoScan[nDeviceIndex].MAC);
        strcpy(sn, gDeviceInfoScan[nDeviceIndex].SN);
        strcpy(version, gDeviceInfoScan[nDeviceIndex].Version);
        strcpy(name, gDeviceInfoScan[nDeviceIndex].Name);
    }
    return 0;
}

int C_PMPAS::emLoadConfigIniFile(EM_DEV_HANDLE hDevice, char* putfilepath)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceLoadConfigini(putfilepath);
}

int C_PMPAS::emRecovConfigIniFile(EM_DEV_HANDLE hDevice)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceRecoveConfigini();
}

int C_PMPAS::emSyncLocalConfigIniFile(EM_DEV_HANDLE hDevice, char *sendfilepath)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceSendConfigini(sendfilepath);
}

void C_PMPAS::doPassData(int nDeviceIndex, unsigned char *buf, unsigned int recvlen)
{
    if(!m_bDevOpen)
        return;
    Embedded_Control* pthis = (Embedded_Control*)devPMP[nDeviceIndex].devHandle;
    pthis->nFrameID ++;
    pthis->tclFrameParam[0].pImgBuf = buf;
    pthis->tclFrameParam[0].pBufferSize = recvlen;
    pthis->m_Ready = true;
}

void C_PMPAS::devPMP_Collect(EM_DEV_HANDLE hDevice, int nDeviceIndex, int isRight)
{
    if(isRight == 0)
    {
        devPMP[nDeviceIndex].devHandle = hDevice;
        devPMP[nDeviceIndex].devInfo = &gDeviceInfoScan[nDeviceIndex];
    }
    else if(isRight == 1)
    {
        devPMP[nDeviceIndex].devHandle = hDevice;
    }
}

int C_PMPAS::emDevStart(EM_DEV_HANDLE hDevice)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceStart();
}

int C_PMPAS::emDevStop(EM_DEV_HANDLE hDevice)
{
    HANDLE_VALIDATE;
    Embedded_Control* pthis = (Embedded_Control*)hDevice;
    return pthis->ctlCmdDeviceStop();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//NETCLASS BASE CLASS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Netclass::Netclass()
{
    memset(IP, C_ZERO, sizeof(IP))              ;
    LocalIP = NULL                              ;
    PORT    = 8180           ;
}

Netclass::~Netclass()
{
}

void Netclass::setCurrentIpAddr(char *ipaddr, const int Port)
{
    this->LocalIP = ipaddr;
    this->PORT = Port;
}

/*
    获取本机IP
*/
int Netclass::getLocalIPs(char ips[][64])
{
    return sys_getlocalIPs(ips);
}

/*
    获取本机IP
*/
int Netclass::sys_getlocalIPs(char ips[][64])
{
    int inet = 0;
    struct ifconf ifconf;
    struct ifreq *ifreq;
    char buf[512];//缓冲区
    //初始化ifconf
    ifconf.ifc_len = 512;
    ifconf.ifc_buf = buf;
//    memset(ifreq->ifr_ifrn.ifrn_name, 0, sizeof(ifreq.ifr_name));
//    strcpy(ifreq->ifr_ifrn.ifrn_name, netcard_name);  //"enp3s0"

    inet  = socket(AF_INET, SOCK_DGRAM, 0);
    if(inet < 0)
    {
        perror("socket");
        exit(1);
    }
    int ret = ioctl(inet, SIOCGIFCONF, &ifconf); //获取所有接口信息
    if(ret!=0)
    {
        perror("GetLocalIp():ioctl error");
        return -1;
    }
    //接下来一个一个的获取IP地址
    ifreq = (struct ifreq*)ifconf.ifc_buf;
//    printf("ifconf.ifc_len:%d\n",ifconf.ifc_len);
//    printf("sizeof (struct ifreq):%ld\n",sizeof(struct ifreq));
    //fflush(stdout);
    unsigned int i = 0;
    for(i = 0;i < ifconf.ifc_len/sizeof(struct ifreq); i++)
    {
        if(ifreq->ifr_flags == AF_INET){ //for ipv4
//            printf("name =[%s]\n" , ifreq->ifr_name);
//            printf("local addr = [%s]\n" ,inet_ntoa(((struct sockaddr_in*)&(ifreq->ifr_addr))->sin_addr));
            strcpy(ips[i], inet_ntoa(((struct sockaddr_in*)&(ifreq->ifr_addr))->sin_addr));
            ifreq++;
        }
//        fflush(stdout);
    }

    close(inet);
    return i;
}

/*
    获取本机MAC
*/
int Netclass::getLocalMAC(char MAC[])
{
    return sys_getlocalMAC(MAC);
}

/*
 * 获取本机MAC
 * */
int Netclass::sys_getlocalMAC(char* MAC)
{
    int sock_mac = socket(AF_INET, SOCK_STREAM, 0);
    if( sock_mac == -1){
        perror("GetLocalMac():create socket fail\n");
        return -1;
    }

    ifreq ifr;
    memset(&ifr,0,sizeof(ifr));
    strcpy(ifr.ifr_name,"eth0");

    int ret =ioctl( sock_mac, SIOCGIFHWADDR, &ifr);
    if( ret < 0 ){
        printf("GetLocalMac():mac ioctl error\n");
        return -2;
    }

    sprintf(MAC, "%02x:%02x:%02x:%02x:%02x:%02x",
            (unsigned char)ifr.ifr_hwaddr.sa_data[0],
            (unsigned char)ifr.ifr_hwaddr.sa_data[1],
            (unsigned char)ifr.ifr_hwaddr.sa_data[2],
            (unsigned char)ifr.ifr_hwaddr.sa_data[3],
            (unsigned char)ifr.ifr_hwaddr.sa_data[4],
            (unsigned char)ifr.ifr_hwaddr.sa_data[5]
            );

    close(sock_mac);

    return 0;
}

int Netclass::sendMsg(int fd, char *buf,int len)
{
    return send(fd, buf, len, 0 );
}

int Netclass::recvMsg(int fd, char *buf,int len)
{
    int buflen = recv(fd, buf, len, 0 );
    return buflen;
}

int Netclass::disconnect(int fd)
{
    int res = close(fd);
//    bool bReuseaddr = true;
//    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&bReuseaddr, sizeof(bool));
    usleep(100 * 1000);
    return res;
}

void Netclass::udpERROR(int error_code)
{
    char str[128] = {0};
    switch(error_code)
    {
    case 1:
        strcpy(str,"UDP缓冲区设置失败!");
        break;
    case 2:
        strcpy(str,"UDP时限设置失败!");
        break;
    case 3:
        strcpy(str,"UDP套接字绑定在一个具体的地址失败!");
        break;
    case 4:
        strcpy(str,"UDP初始化套接字失败!");
        break;
    default:
        break;
    }

   printf("err: %s \n",str);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TCPCLASS CLASS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TCPclass::TCPclass()
{
    fd_tcp = 0;
    bzero((char*)(&serv_tcp), SocketAddressSize);
    bzero((char*)(&client_tcp), SocketAddressSize);
}

int TCPclass::tcpConnect()
{
    //初始化套接字
    this->fd_tcp = socket(AF_INET,SOCK_STREAM,0);
    if(!(this->fd_tcp))
    {
        return -1;
    }

    //然后赋值
    this->serv_tcp.sin_family		=	AF_INET	;
    this->serv_tcp.sin_addr.s_addr	=	inet_addr(LocalIP);	//服务器端的地址
    this->serv_tcp.sin_port			=	htons(PORT)	; 	//服务器端的端口


    //设置为非阻塞模式
    //这样，在connect时，才会立马跳过
    int error = -1 ;
    int err   = -1 ;
    int len   = sizeof(int);

    unsigned long ul = 1;
    ioctl(fd_tcp, (int)FIONBIO, &ul);

    int flags = fcntl(fd_tcp,F_GETFL,0);
    fcntl(fd_tcp,F_SETFL,flags|O_NONBLOCK);

    err = connect(fd_tcp, (struct sockaddr*)&serv_tcp, sizeof(struct sockaddr));
    if(err == -1){
        timeval tm ;
        fd_set  set;
        tm.tv_sec  = 3;
        tm.tv_usec = 0;
        FD_ZERO(&set);
        FD_SET(fd_tcp, &set);
        if( select(fd_tcp + 1, NULL, &set, NULL, &tm) > 0)
        {
                getsockopt(fd_tcp, (int)SOL_SOCKET, (int)SO_ERROR, (char *)&error, (socklen_t *)&len);
                if(error == 0)
                    err = 0;
                else
                    err = -1;
        }
        else
        {
                err = -1;
        }//end if select
    }//end if err

    //设置为阻塞模式
    ul = 0;
    ioctl(fd_tcp, (int)FIONBIO, &ul);

    //时限
    int res = 0 ;
#if defined(_WIN32)
    int nNetTimeout=0.1*1000;//m秒
    res |= setsockopt(this->fd,SOL_SOCKET,SO_SNDTIMEO,(char *)&nNetTimeout,sizeof(int));
    res |= setsockopt(this->fd,SOL_SOCKET,SO_RCVTIMEO,(char *)&nNetTimeout,sizeof(int));
#else
    struct timeval nNetTimeout={4,0};
    res |= setsockopt(this->fd_tcp,SOL_SOCKET,SO_SNDTIMEO,&nNetTimeout,sizeof(nNetTimeout));
    res |= setsockopt(this->fd_tcp,SOL_SOCKET,SO_RCVTIMEO,&nNetTimeout,sizeof(nNetTimeout));
#endif
    if(res<0)
    {
        return -3;
    }

    // 缓冲区
    int nBuf=10*1024*1024;//设置为xM
    res = setsockopt(this->fd_tcp, SOL_SOCKET,SO_SNDBUF,(const char*)&nBuf,sizeof(int));
    res = setsockopt(this->fd_tcp, SOL_SOCKET,SO_RCVBUF,(const char*)&nBuf,sizeof(int));

    if(res)
    {
        printf("tcp : %s setsockopt nBuf error !!\r\n",this->IP);
        return -4;
    }

    return 0 ;
}

int TCPclass::socketConnected()
{
    pid_t pid = getpid();

    struct tcp_info info;
    int len = sizeof(info);
    getsockopt(this->fd_tcp, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len);
    if (info.tcpi_state == TCP_ESTABLISHED)
    {
        return 0;
    }
    else
    {
        printf("pid:%d tcp socket disconnected\n",pid);
        return -1;
    }
}

int TCPclass::tcpSendMsg(char *buf, int len)
{
    return sendMsg(fd_tcp,buf,len);
}

int TCPclass::tcpRecvMsg(char *buf, int len)
{
    return recvMsg(fd_tcp,buf,len);
}

int TCPclass::tcpDisconnect()
{
    return disconnect(fd_tcp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// IMAGEFIFO CLASS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ImageFIFO::NaldataFIFO_Init()
{
    this->error = 0 ;
    for(int i=0;i<8;i++){
            ImageFIFO::ppNal_buff[i] = new unsigned char[width*height*channel+4+64];
            if(ImageFIFO::ppNal_buff[i] == NULL){
                ImageFIFO::error = 1 ;
            }

    }

    pthread_mutex_init(&Q_Mutex, NULL);

    nal_r_n = 0 ;
    nal_w_n = 0 ;
    nal_r_i = 0 ;
    nal_w_i = 0 ;
    lost_RDP_pkg_num = 0 ;
    lost_now = 0 ;

    return ImageFIFO::error ;
}

int ImageFIFO::NaldataFIFO_Release()
{

    for(int i=7;i>=0;i--){
        delete ImageFIFO::ppNal_buff[i];
        ImageFIFO::ppNal_buff[i] = NULL ;
    }

    nal_r_n = 0 ;
    nal_w_n = 0 ;
    nal_r_i = 0 ;
    nal_w_i = 0 ;

    pthread_mutex_destroy(&Q_Mutex);

    return 0 ;
}

int ImageFIFO::NaldataFIFO_PUSH(unsigned char *nal, int len)
{

    pthread_mutex_lock(&Q_Mutex);

    unsigned char *point = ppNal_buff[nal_w_i];

    *(int *)point = len ;
    memcpy(point, nal, len);

    if(nal_w_n<7){
        nal_w_n++;
        lost_now = 0 ;
    }
    else{
        nal_r_i++;
        nal_r_i &= 0x07 ;
        lost_RDP_pkg_num = lost_RDP_pkg_num +1.0 ;
        lost_now = 1 ;
    }

    nal_w_i++;
    nal_w_i &= 0x07 ;

    pthread_mutex_unlock(&Q_Mutex);

    return 0 ;
}

int ImageFIFO::NaldataFIFO_POP(unsigned char **nal, int *plen)
{
    pthread_mutex_lock(&Q_Mutex);

    *plen = 0 ;
    unsigned char *point = ppNal_buff[nal_r_i];

    if(nal_w_n>0){

        *plen = *(int *)point;

        memcpy(*nal, point, *plen);

        nal_r_i++;
        nal_r_i &= 0x07 ;
        nal_r_n++;
        nal_w_n--;
    }

    pthread_mutex_unlock(&Q_Mutex);

    return 0 ;
}

void Embedded_Control::pthread_suspend(char* pid)
{
    if (pthread_pause == false)
    {
        pthread_mutex_lock( &mutex);
        pthread_pause = true;
        printf("%s -->pthread pause------\n", pid);
        pthread_mutex_unlock( &mutex );
    }
    else
    {
        printf("%s -->pthread suspend already.\n", pid);
    }
}

void Embedded_Control::pthread_resume(char* pid)
{
    if (pthread_pause == true)
    {
        pthread_mutex_lock(&mutex);
        pthread_pause = false;
        pthread_cond_broadcast(&cond);
        printf("%s -->pthread resume------\n", pid);
        pthread_mutex_unlock(&mutex);
    }
    else
    {
        printf("%s -->pthread resume already.\n", pid);
    }
}

void Embedded_Control::pthread_pause_location(void)
{
    pthread_mutex_lock(&mutex);
    while(this->pthread_pause)
    {
        pthread_cond_wait(&cond, &mutex);
    }
    pthread_mutex_unlock(&mutex);
}




