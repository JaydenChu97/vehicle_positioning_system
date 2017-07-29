#ifndef SAMPLEAPP_H
#define SAMPLEAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

#define SAMPLEAPP_ENDPOINT           20

#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

//主任务的最大功能模块数
#define SAMPLEAPP_MAX_CLUSTERS       1
#define SAMPLEAPP_PERIODIC_CLUSTERID 1


//发送信息时间间隔(单位毫秒)
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT   1000    

//发送信息函数事件标志
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
  


/*********************************************************************
 * FUNCTIONS
 */


//主任务相关初始化
extern void SampleApp_Init( uint8 task_id );

//主任务处理函数 
extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */
