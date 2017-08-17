/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "OnBoard.h"

#include "SampleApp.h"
#include "GPS_Data_Process.h"


/* HAL */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */

// 任务蔟号, 注册某一任务的某一功能
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
    SAMPLEAPP_PERIODIC_CLUSTERID,
};

//设备的简单描述符
const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
    SAMPLEAPP_ENDPOINT,              //  int Endpoint;
    SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
    SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
    SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
    SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
    SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
    (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // 任务号, 任务号越小优先级越高, 在SampleApp_Init()被调用时初始化
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // 消息标识码

afAddrType_t SampleApp_Periodic_DstAddr;

uint8 SampleAppPeriodicCounter = 0;

GeoBuf geoBuf;
GeoInfo geoInfo;
//uint8 geoInfoData[GEO_INFO_NUM][20];
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( uint8 *data, uint16 len );

void initUart(int port, void (*callBack)(uint8, uint8));
void initGeoBuf(void);

void recGPSData(uint8 port, uint8 event);
//void fetchGPSInfo(void);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * 初始化函数
 */
//串口初始化(DMA传送模式和ISR模式)
void initUart(int port, void (*callBack)(uint8, uint8))
{
    //UART设置
    halUARTCfg_t uartConfig;
    uartConfig.configured           = TRUE;             //设置有效标志
    uartConfig.baudRate             = HAL_UART_BR_9600;//波特率设置
    uartConfig.flowControl          = FALSE;           //关闭流控制(流控制需要四线传输,加上CTS,RTS)
    uartConfig.rx.maxBufSize        = 512;            //rx缓冲区的长度
    uartConfig.tx.maxBufSize        = 512;            //tx缓冲区的长度
    uartConfig.idleTimeout          = 6;              //RX为空的允许时间
    uartConfig.intEnable            = TRUE;           //中断允许
    uartConfig.callBackFunc         = callBack;     //当出现回调事件时的回调函数
                                                      //回调事件列表:
                                                      //HAL_UART_RX_FULL        RX满
                                                      //HAL_UART_RX_ABOUT_FULL  RX超过流量控制阀值
                                                      //HAL_UART_RX_TIMEOUT     RX为空的时间超过限制
                                                      //HAL_UART_TX_FULL        TX满
    
    HalUARTOpen (port, &uartConfig);
    HalUARTWrite(0,"Link Start!\n", sizeof("Link Start!\n")-1);//连接上位机成功提示信息
}

//GPS缓冲池初始化
void initGeoBuf(void)
{
    geoBuf.head = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
    osal_memset(geoBuf.head, 0, sizeof(DataFrame));
    geoBuf.send = geoBuf.rec = geoBuf.head;
    geoBuf.checkByte = geoBuf.AIM = "$GPGGA";               //目标提取信息前缀为$GPGGA
}


/*********************************************************************
 * 数据处理函数
 */
//回调函数, 当RX缓冲区满后会调用, 将目标GPS数据粗提取并存入缓冲池中, 留待处理
void recGPSData(uint8 port, uint8 event)
{
    uint8  ch;

    while (Hal_UART_RxBufLen(port))
    {
        HalUARTRead (port, &ch, 1);
        
        if(geoBuf.mode == 0)
        {
            if(ch == *geoBuf.checkByte)
            {
                geoBuf.checkByte++;
            }
            else
            {
                geoBuf.checkByte = geoBuf.AIM;
            }
            
            if(geoBuf.checkByte == geoBuf.AIM + 6)
            {
                geoBuf.mode = 1;
            }
        }
        else
        {
            if(ch == '$')
            {
                if(geoBuf.rec->len >= 64)           //通过两个$符号之间的数据长度确定数据的有效性
                {
                    geoBuf.rec->next = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
                    geoBuf.rec = geoBuf.rec->next;
                    osal_memset(geoBuf.rec, 0, sizeof(DataFrame));
                    geoBuf.num++;
                }
                else
                {
                    //SampleApp_SendPeriodicMessage("Connect Fail! ---- ", 19);          //无效GPS数据
                    SampleApp_SendPeriodicMessage(geoBuf.rec->data, geoBuf.rec->len);
                    geoBuf.rec->len = 0;
                }
                
                geoBuf.mode = 0;
            }
            else
            {
                geoBuf.rec->data[geoBuf.rec->len++] = ch;        
            }
        }  
    }
} 
 
//从每一个有效数据帧中提取所需地理信息, 并且刷新geoInfoData对象, 同时将数据封装到一个包中用于发送
// void fetchGPSInfo(void)
// {
    // uint16 i = 0, j = 0;
    
    // uint8* ptr = (void*)&geoInfo;        //将整个结构体视为一个由字节构成的整体处理
    
    // while(i <= sizeof(geoInfo))
    // {
        // if(geoBuf.send->data[j] == ',')
        // {
            // j++;
            // continue;
        // }
        
        // ptr[i++] = geoBuf.send->data[j++];
    // }
    
// }
/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{ 
    SampleApp_TaskID = task_id;
    SampleApp_NwkState = DEV_INIT;
    SampleApp_TransID = 0;
  
    initUart(0, (void*)0);      //初始化与上位机相连的串口
    initUart(1, recGPSData);    //初始化与GPS相连的串口
    initGeoBuf();

    // 设置消息发送目标地址(广播模式)
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
    
    // 填充端点描述符
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_epDesc.task_id = &SampleApp_TaskID;
    SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    SampleApp_epDesc.latencyReq = noLatencyReqs;
    
    // 向应用层注册端点
    afRegister( &SampleApp_epDesc );
    
    // 表明这个APP处理所有事务
    RegisterForKeys( SampleApp_TaskID );
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
    afIncomingMSGPacket_t *MSGpkt;
    (void)task_id;  // Intentionally unreferenced parameter
    
    if ( events & SYS_EVENT_MSG ) //检测对应位是否为1以判定是否为是相应事件, 其余位一般为零
    {
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        
        while ( MSGpkt )
        {
            switch ( MSGpkt->hdr.event )
            {        
                // Received when a messages is received (OTA) for this endpoint
                case AF_INCOMING_MSG_CMD:
                    SampleApp_MessageMSGCB( MSGpkt );
                    break;
            
                // 网络状态改变触发此事件, 也就意味着只有当建网成功后才会开始间隔发送消息
                case ZDO_STATE_CHANGE:
                    SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                        (SampleApp_NwkState == DEV_ROUTER)
                        || (SampleApp_NwkState == DEV_END_DEVICE) )
                    {
                    // 从现在开始计时, 到规定时间即向指定任务发送时间标志, 只执行一次
                    osal_start_timerEx( SampleApp_TaskID,
                                        SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                        SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                    }
                    else
                    {
                    // Device is no longer in the network
                    }
                    break;
            
                default:
                    break;
            }
        
            // 释放空间
            osal_msg_deallocate( (uint8 *)MSGpkt );
        
            // 如果缓冲区还有剩余的事件则继续处理
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        }
        
        // 返回未处理的事件
        return (events ^ SYS_EVENT_MSG);
    }
    
    // 设置发送消息的时间间隔, 由计时器触发
    if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
    {
 
        //限制只有至少存在两个有效数据帧时再处理, 防止数据提取速率超过接收速率, 破坏链表结构
        if(geoBuf.num >= 2)  
        {

            //fetchGPSInfo();
            
            HalUARTWrite(0, "LocalHost : ", sizeof("LocalHost : ")-1); //发送自机数据
            HalUARTWrite(0, geoBuf.send->data, geoBuf.send->len); //发送自机数据
            
            SampleApp_SendPeriodicMessage(geoBuf.send->data, geoBuf.send->len);

            geoBuf.num--;
            
            geoBuf.send = geoBuf.send->next;
            
            osal_mem_free(geoBuf.head);
            geoBuf.head = geoBuf.send;   
        }
        
        // 设置事件触发的时间间隔
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
            SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT);
        
        // 返回未处理的事件
        return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
    }
    
    //丢弃无用事件
    return 0;
}

/*********************************************************************
 * Event Generation Functions
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   处理别的结点发来的数据
 *          
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8 num[20];
  switch ( pkt->clusterId )
  {
        case SAMPLEAPP_PERIODIC_CLUSTERID:
        //输出结点所属网络PAN ID, 一个结点可以同时处在多个网络中, 用PAN ID进行区分
        _ltoa(pkt->srcAddr.panId, num, 16);
        HalUARTWrite(0, "0x", 2);        
        HalUARTWrite(0, num, osal_strlen(num));
        
        //输出网络短地址, 相当于IP地址
        osal_memset(num, 0, sizeof num);        //初始化缓冲存储空间
        _ltoa((uint16)(pkt->srcAddr.addr.shortAddr), num, 16);   //数字转字符串
        HalUARTWrite(0, "->0x", 4);  
        HalUARTWrite(0, num, osal_strlen(num));                 //提示信息(消息来源端点号)
        HalUARTWrite(0, " : ", 3);                              //提示信息
        
        // if(pkt->cmd.DataLength == sizeof(geoInfo))
        // {
            // uint16 i = 0;
            // for(; i < 5; i++)
            // {
                // HalUARTWrite(0, "\n   ", 4);  
                // HalUARTWrite(0, GeoInfoName[i], osal_strlen(GeoInfoName[i]));
                // HalUARTWrite(0, pkt->cmd.Data + geoInfoLen[i], geoInfoLen[i+1] - geoInfoLen[i]);    //输出接收到的数据
            // }
            
            // HalUARTWrite(0, "\n", 1);                               //回车换行
        // }
        // else
        {
            HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);    //输出接收到的数据
        }
        
        break;

  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   定时广播自身结点数据
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage(uint8 *data, uint16 len)
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       len,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
