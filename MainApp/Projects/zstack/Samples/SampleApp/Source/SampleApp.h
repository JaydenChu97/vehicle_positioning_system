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

//任务端点号, 相当于端口
#define SAMPLEAPP_ENDPOINT           20

//固定选项, 无需改动
#define SAMPLEAPP_PROFID             0x0F08
#define SAMPLEAPP_DEVICEID           0x0001
#define SAMPLEAPP_DEVICE_VERSION     0
#define SAMPLEAPP_FLAGS              0

//任务功能蔟
#define SAMPLEAPP_MAX_CLUSTERS       1
#define SAMPLEAPP_PERIODIC_CLUSTERID 1

//消息发送时间间隔
#define SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT   1000    
//消息发送事件标志
#define SAMPLEAPP_SEND_PERIODIC_MSG_EVT       0x0001
  


/*********************************************************************
 * FUNCTIONS
 */


extern void SampleApp_Init( uint8 task_id );

extern UINT16 SampleApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SAMPLEAPP_H */
