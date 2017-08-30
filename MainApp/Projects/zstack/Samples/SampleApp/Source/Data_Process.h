#ifndef DATA_PROCESS
#define DATA_PROCESS

#include "OSAL.h"
#include "hal_types.h"
#include "hal_uart.h"

/****************************************************************
缓冲池中的每一个有效数据帧, 存储GPS协议中要提取的信息, 
并且构成一个链表				
****************************************************************/
typedef struct
{
    uint16 len;               //已经提取的数据长度
    uint8 data[80];         //数据内容
}DataFrame;

/****************************************************************
缓冲池结构体, 控制缓冲池状态
****************************************************************/
struct
{   
    DataFrame* send;        //指向已经填充完毕的数据帧
    DataFrame* rec;         //指向当前需要填充数据的数据帧
    
    uint8* AIM;             //目标提取信息关键词
    uint8* checkByte;       //与目标关键词匹配成功的当前位置
    uint16 mode;              //缓冲池模式, 0为检测模式(等待有效数据)
                            //1为接收模式, 存储每一个接收到的数据  
}geoBuf;//gps数据缓冲池

void initGeoBuf(void);
void recGPSData(uint8 port, uint8 event);

void IEEE64AddrToStr(uint8* source, uint8* dest);

//GPS缓冲池初始化
void initGeoBuf(void)
{
    geoBuf.rec = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
    osal_memset(geoBuf.rec, 0, sizeof(DataFrame));
    geoBuf.send = NULL;
    geoBuf.checkByte = geoBuf.AIM = "$GPGGA";               //目标提取信息前缀为$GPGGA
}

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
                    geoBuf.send = geoBuf.rec;
                    geoBuf.rec = (DataFrame*)osal_mem_alloc(sizeof(DataFrame));
                    osal_memset(geoBuf.rec, 0, sizeof(DataFrame));
                }
                else
                {
                    //SampleApp_SendPeriodicMessage(geoBuf.rec->data, geoBuf.rec->len);
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

//将IEEE64位地址转换为16位字符串形式
void IEEE64AddrToStr(uint8* source, uint8* dest)
{
    osal_memset(dest, '0', 16);
    
    uint8 i = 0;
    while(i < 8)
    {
        uint8 first = source[i]/16;
        uint8 second = source[i]%16;

        if(first >= 0 && first <= 9)
        {
            dest[3*i] = '0' + first;
        }
        else
        {
            dest[3*i] = 'A' + first - 10;
        }

        if(second >= 0 && second <= 9)
        {
            dest[3*i+1] = '0' + second;
        }
        else
        {
            dest[3*i+1] = 'A' + second - 10;
        }

        dest[3*i+2] = ':';

        i++;
    }
}

#endif