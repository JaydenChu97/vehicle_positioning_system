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

afAddrType_t SampleApp_Periodic_DstAddr;    //����Ŀ���ַ������

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( uint8 *data, uint16 len );

void initLed(void);
void initUart(uint8 port, void (*callBack)(uint8, uint8), uint16 baudRate);

/*********************************************************************
 * ��ʼ������
 */
// LED��ʼ��, ���ڱ�ʾ�豸������״̬
void initLed(void)
{
    HalLedInit();
    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);   //δ����ǰΪ�ر�״̬, ��������
    HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);   //������Ϣʱ��˸
    HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);   //������Ϣʱ��˸
    
}
 
//���ڳ�ʼ��(DMA����ģʽ��ISRģʽ)
void initUart(uint8 port, void (*callBack)(uint8, uint8), uint16 baudRate)
{
    //UART����
    halUARTCfg_t uartConfig;
    uartConfig.configured           = TRUE;             //������Ч��־
    uartConfig.baudRate             = baudRate;         //����������
    uartConfig.flowControl          = FALSE;           //�ر�������(��������Ҫ���ߴ���,����CTS,RTS)
    uartConfig.rx.maxBufSize        = 256;            //rx�������ĳ���
    uartConfig.tx.maxBufSize        = 256;            //tx�������ĳ���
    uartConfig.idleTimeout          = 6;              //RXΪ�յ�����ʱ��
    uartConfig.intEnable            = TRUE;           //�ж�����
    uartConfig.callBackFunc         = callBack;       //�����ֻص��¼�ʱ�Ļص�����
                                                      //�ص��¼��б�:
                                                      //HAL_UART_RX_FULL        RX��
                                                      //HAL_UART_RX_ABOUT_FULL  RX�����������Ʒ�ֵ
                                                      //HAL_UART_RX_TIMEOUT     RXΪ�յ�ʱ�䳬������
                                                      //HAL_UART_TX_FULL        TX��
    
    HalUARTOpen (port, &uartConfig);
}


/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   ��ʼ�������������Ӳ�����豸״̬
 *
 * @param   task_id ��Ϣ���͵�Ŀ������ID
 *
 * @return   ��
 */
void SampleApp_Init(uint8 task_id)
{ 
    SampleApp_TaskID = task_id;
    SampleApp_NwkState = DEV_INIT;  //��ʼ���豸����״̬
    SampleApp_TransID = 0;
    
    initLed();                  //��ʼ��LED��
    initUart(0, NULL, HAL_UART_BR_115200);          //��ʼ������λ�������Ĵ���
    
    #ifdef ZDO_COORDINATOR
        initESP();
    #else
        initUart(1, recGPSData, HAL_UART_BR_9600);    //��ʼ����GPS�����Ĵ���
        initGeoBuf();               //��ʼ��GPS���ݻ����
    #endif
    
    // ������Ϣ����Ŀ���ַ(�㲥ģʽ)
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
    
    // ���˵�������
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_epDesc.task_id = &SampleApp_TaskID;
    SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    SampleApp_epDesc.latencyReq = noLatencyReqs;        //��ѡ��
    
    // ��Ӧ�ò�ע��˵�
    afRegister( &SampleApp_epDesc );
    
    // �������APP������������
    RegisterForKeys( SampleApp_TaskID );
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   ��������������ص��¼�
 *
 * @param   task_id ��Ϣ���͵�Ŀ������ID
 * @param   events ������Ҫ������¼�, ����ͬʱ�����������, ÿһλ�����Ա�ʾ������Ϣ
 *
 * @return  ��
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
                // ���յ���Ľ�㷢�͹�������Ϣʱ, ��������¼�
                case AF_INCOMING_MSG_CMD:
                    #ifdef ZDO_COORDINATOR
                        uploadData(MSGpkt);
                    #else
                        SampleApp_MessageMSGCB(MSGpkt);
                    #endif
                    HalLedBlink(HAL_LED_3, 2, 50, 500); // LED3�� 2��, ����ʱ��ռ��˸���ڵ�50%, ����Ϊ500����
                    
                    break;
            
                // ����״̬�ı䴥�����¼�, Ҳ����ζ��ֻ�е������ɹ���ŻῪʼ���������Ϣ
                case ZDO_STATE_CHANGE:
                    SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||
                        (SampleApp_NwkState == DEV_ROUTER)
                        || (SampleApp_NwkState == DEV_END_DEVICE) )
                    {
                        // �����罨���ɹ�ʱ
                        HalUARTWrite(0, "Link Start!\n", osal_strlen("Link Start!\n"));
                        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);  //LED����״̬
                        
                        // �����ڿ�ʼ��ʱ, ���涨ʱ�伴��ָ��������ʱ���־, ִֻ��һ��
                        osal_start_timerEx( SampleApp_TaskID,
                                            SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                            SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                    }
                    else
                    {
                        // ���豸��������ʱ
                        #ifndef ZDO_COORDINATOR
                            HalUARTWrite(0, "Lose Connection!\n", osal_strlen("Lose Connection!\n"));
                            HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
                        #endif
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
 
        //���пɷ��͵�����ʱ������
        if(geoBuf.send != NULL)
        {
            uint8 data[100];
            uint8 len = 0;
            
            //���IEEE64λ��ַ
            osal_memcpy(data, aExtendedAddress, Z_EXTADDR_LEN);
            len += Z_EXTADDR_LEN;
            
            //���GPS����
            osal_memcpy(data + len, geoBuf.send->data, geoBuf.send->len);
            len += geoBuf.send->len;
            osal_mem_free(geoBuf.send);
            geoBuf.send = NULL; 
            
            HalLedBlink(HAL_LED_2, 2, 50, 500); // LED2�� 2��, ����ʱ��ռ��˸���ڵ�50%, ����Ϊ500����
            
            //�����Ի�����
            HalUARTWrite(0, "LocalHost : ", osal_strlen("LocalHost : "));
            HalUARTWrite(0, data, len);       
            
            SampleApp_SendPeriodicMessage(data, len);
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
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   �����Ľ�㷢��������
 *          
 * @param   ��
 *
 * @return  ��
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
        HalUARTWrite(0, "(", 1);                              //��ʾ��Ϣ
        
        //����豸��IEEE64λ��ַ, ����Ψһȷ��ÿ����Ϣ��������
        uint8 IEEE64Addr[23];
        IEEE64AddrToStr(pkt->cmd.Data, IEEE64Addr);
        HalUARTWrite(0, IEEE64Addr, sizeof IEEE64Addr);
        HalUARTWrite(0, ")", 1); 

        HalUARTWrite(0, pkt->cmd.Data+8, pkt->cmd.DataLength-8);    //������յ�������
        
        break;

  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   ��ʱ�㲥����������
 *
 * @param   ��
 *
 * @return  ��
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
        //��Ϣ����ʧ��
        HalUARTWrite(0, "Send message fail!\n", osal_strlen("Send message fail!\n"));
  }
}
