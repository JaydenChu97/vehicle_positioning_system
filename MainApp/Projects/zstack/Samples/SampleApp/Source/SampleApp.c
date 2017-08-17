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

// ��������, ע��ĳһ�����ĳһ����
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
    SAMPLEAPP_PERIODIC_CLUSTERID,
};

//�豸�ļ�������
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
uint8 SampleApp_TaskID;   // �����, �����ԽС���ȼ�Խ��, ��SampleApp_Init()������ʱ��ʼ��
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // ��Ϣ��ʶ��

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
 * ��ʼ������
 */
//���ڳ�ʼ��(DMA����ģʽ��ISRģʽ)
void initUart(int port, void (*callBack)(uint8, uint8))
{
    //UART����
    halUARTCfg_t uartConfig;
    uartConfig.configured           = TRUE;             //������Ч��־
    uartConfig.baudRate             = HAL_UART_BR_9600;//����������
    uartConfig.flowControl          = FALSE;           //�ر�������(��������Ҫ���ߴ���,����CTS,RTS)
    uartConfig.rx.maxBufSize        = 512;            //rx�������ĳ���
    uartConfig.tx.maxBufSize        = 512;            //tx�������ĳ���
    uartConfig.idleTimeout          = 6;              //RXΪ�յ�����ʱ��
    uartConfig.intEnable            = TRUE;           //�ж�����
    uartConfig.callBackFunc         = callBack;     //�����ֻص��¼�ʱ�Ļص�����
                                                      //�ص��¼��б�:
                                                      //HAL_UART_RX_FULL        RX��
                                                      //HAL_UART_RX_ABOUT_FULL  RX�����������Ʒ�ֵ
                                                      //HAL_UART_RX_TIMEOUT     RXΪ�յ�ʱ�䳬������
                                                      //HAL_UART_TX_FULL        TX��
    
    HalUARTOpen (port, &uartConfig);
    HalUARTWrite(0,"Link Start!\n", sizeof("Link Start!\n")-1);//������λ���ɹ���ʾ��Ϣ
}

//GPS����س�ʼ��
void initGeoBuf(void)
{
    geoBuf.head = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
    osal_memset(geoBuf.head, 0, sizeof(DataFrame));
    geoBuf.send = geoBuf.rec = geoBuf.head;
    geoBuf.checkByte = geoBuf.AIM = "$GPGGA";               //Ŀ����ȡ��Ϣǰ׺Ϊ$GPGGA
}


/*********************************************************************
 * ���ݴ�����
 */
//�ص�����, ��RX��������������, ��Ŀ��GPS���ݴ���ȡ�����뻺�����, ��������
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
                if(geoBuf.rec->len >= 64)           //ͨ������$����֮������ݳ���ȷ�����ݵ���Ч��
                {
                    geoBuf.rec->next = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
                    geoBuf.rec = geoBuf.rec->next;
                    osal_memset(geoBuf.rec, 0, sizeof(DataFrame));
                    geoBuf.num++;
                }
                else
                {
                    //SampleApp_SendPeriodicMessage("Connect Fail! ---- ", 19);          //��ЧGPS����
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
 
//��ÿһ����Ч����֡����ȡ���������Ϣ, ����ˢ��geoInfoData����, ͬʱ�����ݷ�װ��һ���������ڷ���
// void fetchGPSInfo(void)
// {
    // uint16 i = 0, j = 0;
    
    // uint8* ptr = (void*)&geoInfo;        //�������ṹ����Ϊһ�����ֽڹ��ɵ����崦��
    
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
  
    initUart(0, (void*)0);      //��ʼ������λ�������Ĵ���
    initUart(1, recGPSData);    //��ʼ����GPS�����Ĵ���
    initGeoBuf();

    // ������Ϣ����Ŀ���ַ(�㲥ģʽ)
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
    
    // ���˵�������
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_epDesc.task_id = &SampleApp_TaskID;
    SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    SampleApp_epDesc.latencyReq = noLatencyReqs;
    
    // ��Ӧ�ò�ע��˵�
    afRegister( &SampleApp_epDesc );
    
    // �������APP������������
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
    
    if ( events & SYS_EVENT_MSG ) //����Ӧλ�Ƿ�Ϊ1���ж��Ƿ�Ϊ����Ӧ�¼�, ����λһ��Ϊ��
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
            
                // ����״̬�ı䴥�����¼�, Ҳ����ζ��ֻ�е������ɹ���ŻῪʼ���������Ϣ
                case ZDO_STATE_CHANGE:
                    SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                        (SampleApp_NwkState == DEV_ROUTER)
                        || (SampleApp_NwkState == DEV_END_DEVICE) )
                    {
                    // �����ڿ�ʼ��ʱ, ���涨ʱ�伴��ָ��������ʱ���־, ִֻ��һ��
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
        
            // �ͷſռ�
            osal_msg_deallocate( (uint8 *)MSGpkt );
        
            // �������������ʣ����¼����������
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        }
        
        // ����δ������¼�
        return (events ^ SYS_EVENT_MSG);
    }
    
    // ���÷�����Ϣ��ʱ����, �ɼ�ʱ������
    if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
    {
 
        //����ֻ�����ٴ���������Ч����֡ʱ�ٴ���, ��ֹ������ȡ���ʳ�����������, �ƻ�����ṹ
        if(geoBuf.num >= 2)  
        {

            //fetchGPSInfo();
            
            HalUARTWrite(0, "LocalHost : ", sizeof("LocalHost : ")-1); //�����Ի�����
            HalUARTWrite(0, geoBuf.send->data, geoBuf.send->len); //�����Ի�����
            
            SampleApp_SendPeriodicMessage(geoBuf.send->data, geoBuf.send->len);

            geoBuf.num--;
            
            geoBuf.send = geoBuf.send->next;
            
            osal_mem_free(geoBuf.head);
            geoBuf.head = geoBuf.send;   
        }
        
        // �����¼�������ʱ����
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
            SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT);
        
        // ����δ������¼�
        return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
    }
    
    //���������¼�
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
 * @brief   �����Ľ�㷢��������
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
        //��������������PAN ID, һ��������ͬʱ���ڶ��������, ��PAN ID��������
        _ltoa(pkt->srcAddr.panId, num, 16);
        HalUARTWrite(0, "0x", 2);        
        HalUARTWrite(0, num, osal_strlen(num));
        
        //�������̵�ַ, �൱��IP��ַ
        osal_memset(num, 0, sizeof num);        //��ʼ������洢�ռ�
        _ltoa((uint16)(pkt->srcAddr.addr.shortAddr), num, 16);   //����ת�ַ���
        HalUARTWrite(0, "->0x", 4);  
        HalUARTWrite(0, num, osal_strlen(num));                 //��ʾ��Ϣ(��Ϣ��Դ�˵��)
        HalUARTWrite(0, " : ", 3);                              //��ʾ��Ϣ
        
        // if(pkt->cmd.DataLength == sizeof(geoInfo))
        // {
            // uint16 i = 0;
            // for(; i < 5; i++)
            // {
                // HalUARTWrite(0, "\n   ", 4);  
                // HalUARTWrite(0, GeoInfoName[i], osal_strlen(GeoInfoName[i]));
                // HalUARTWrite(0, pkt->cmd.Data + geoInfoLen[i], geoInfoLen[i+1] - geoInfoLen[i]);    //������յ�������
            // }
            
            // HalUARTWrite(0, "\n", 1);                               //�س�����
        // }
        // else
        {
            HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);    //������յ�������
        }
        
        break;

  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   ��ʱ�㲥����������
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
