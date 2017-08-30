/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "OnBoard.h"

#include "hal_led.h"

#include "SampleApp.h"
#include "Data_Process.h"
#include "Ethernet_Config.h"

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

afAddrType_t SampleApp_Periodic_DstAddr;    //发送目标地址描述符

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( uint8 *data, uint16 len );

void initLed(void);
void initUart(uint8 port, void (*callBack)(uint8, uint8), uint16 baudRate);

/*********************************************************************
 * 初始化函数
 */
// LED初始化, 用于表示设备的网络状态
void initLed(void)
{
    HalLedInit();
    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);   //未入网前为关闭状态, 入网后长亮
    HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);   //发送信息时闪烁
    HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);   //接受信息时闪烁
    
}
 
//串口初始化(DMA传送模式和ISR模式)
void initUart(uint8 port, void (*callBack)(uint8, uint8), uint16 baudRate)
{
    //UART设置
    halUARTCfg_t uartConfig;
    uartConfig.configured           = TRUE;             //设置有效标志
    uartConfig.baudRate             = baudRate;         //波特率设置
    uartConfig.flowControl          = FALSE;           //关闭流控制(流控制需要四线传输,加上CTS,RTS)
    uartConfig.rx.maxBufSize        = 256;            //rx缓冲区的长度
    uartConfig.tx.maxBufSize        = 256;            //tx缓冲区的长度
    uartConfig.idleTimeout          = 6;              //RX为空的允许时间
    uartConfig.intEnable            = TRUE;           //中断允许
    uartConfig.callBackFunc         = callBack;       //当出现回调事件时的回调函数
                                                      //回调事件列表:
                                                      //HAL_UART_RX_FULL        RX满
                                                      //HAL_UART_RX_ABOUT_FULL  RX超过流量控制阀值
                                                      //HAL_UART_RX_TIMEOUT     RX为空的时间超过限制
                                                      //HAL_UART_TX_FULL        TX满
    
    HalUARTOpen (port, &uartConfig);
}


/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   初始化所有任务相关硬件和设备状态
 *
 * @param   task_id 消息发送的目标任务ID
 *
 * @return   无
 */
void SampleApp_Init(uint8 task_id)
{ 
    SampleApp_TaskID = task_id;
    SampleApp_NwkState = DEV_INIT;  //初始化设备网络状态
    SampleApp_TransID = 0;
    
    initLed();                  //初始化LED灯
    initUart(0, NULL, HAL_UART_BR_115200);          //初始化与上位机相连的串口
    
    #ifdef ZDO_COORDINATOR
        initESP();
    #else
        initUart(1, recGPSData, HAL_UART_BR_9600);    //初始化与GPS相连的串口
        initGeoBuf();               //初始化GPS数据缓冲池
    #endif
    
    // 设置消息发送目标地址(广播模式)
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
    
    // 填充端点描述符
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_epDesc.task_id = &SampleApp_TaskID;
    SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    SampleApp_epDesc.latencyReq = noLatencyReqs;        //必选项
    
    // 向应用层注册端点
    afRegister( &SampleApp_epDesc );
    
    // 表明这个APP处理所有事务
    RegisterForKeys( SampleApp_TaskID );
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   处理所有任务相关的事件
 *
 * @param   task_id 消息发送的目标任务ID
 * @param   events 任务需要处理的事件, 可以同时包含多个任务, 每一位都可以表示任务信息
 *
 * @return  无
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
                // 当收到别的结点发送过来的消息时, 触发这个事件
                case AF_INCOMING_MSG_CMD:
                    #ifdef ZDO_COORDINATOR
                        uploadData(MSGpkt);
                    #else
                        SampleApp_MessageMSGCB(MSGpkt);
                    #endif
                    HalLedBlink(HAL_LED_3, 2, 50, 500); // LED3亮 2次, 暗的时间占闪烁周期的50%, 周期为500毫秒
                    
                    break;
            
                // 网络状态改变触发此事件, 也就意味着只有当建网成功后才会开始间隔发送消息
                case ZDO_STATE_CHANGE:
                    SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                        (SampleApp_NwkState == DEV_ROUTER)
                        || (SampleApp_NwkState == DEV_END_DEVICE) )
                    {
                        // 当网络建立成功时
                        HalUARTWrite(0, "Link Start!\n", osal_strlen("Link Start!\n"));
                        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);  //LED长亮状态
                        
                        // 从现在开始计时, 到规定时间即向指定任务发送时间标志, 只执行一次
                        osal_start_timerEx( SampleApp_TaskID,
                                            SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                            SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                    }
                    else
                    {
                        // 当设备脱离网络时
                        #ifndef ZDO_COORDINATOR
                            HalUARTWrite(0, "Lose Connection!\n", osal_strlen("Lose Connection!\n"));
                            HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
                        #endif
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
 
        //当有可发送的数据时即发送
        if(geoBuf.send != NULL)
        {
            uint8 data[100];
            uint8 len = 0;
            
            //填充IEEE64位地址
            osal_memcpy(data, aExtendedAddress, Z_EXTADDR_LEN);
            len += Z_EXTADDR_LEN;
            
            //填充GPS数据
            osal_memcpy(data + len, geoBuf.send->data, geoBuf.send->len);
            len += geoBuf.send->len;
            osal_mem_free(geoBuf.send);
            geoBuf.send = NULL; 
            
            HalLedBlink(HAL_LED_2, 2, 50, 500); // LED2亮 2次, 暗的时间占闪烁周期的50%, 周期为500毫秒
            
            //发送自机数据
            HalUARTWrite(0, "LocalHost : ", osal_strlen("LocalHost : "));
            HalUARTWrite(0, data, len);       
            
            SampleApp_SendPeriodicMessage(data, len);
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
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   处理别的结点发来的数据
 *          
 * @param   无
 *
 * @return  无
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
        HalUARTWrite(0, "(", 1);                              //提示信息
        
        //输出设备的IEEE64位地址, 用以唯一确定每条信息所属车辆
        uint8 IEEE64Addr[23];
        IEEE64AddrToStr(pkt->cmd.Data, IEEE64Addr);
        HalUARTWrite(0, IEEE64Addr, sizeof IEEE64Addr);
        HalUARTWrite(0, ")", 1); 

        HalUARTWrite(0, pkt->cmd.Data+8, pkt->cmd.DataLength-8);    //输出接收到的数据
        
        break;

  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   定时广播自身结点数据
 *
 * @param   无
 *
 * @return  无
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
        //消息发送失败
        HalUARTWrite(0, "Send message fail!\n", osal_strlen("Send message fail!\n"));
  }
}
