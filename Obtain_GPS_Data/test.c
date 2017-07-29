#include <iocc2530.h>
#include <stdlib.h>
#include <string.h>

#include "Init.h"

#define uint unsigned int
#define uchar unsigned char

/****************************************************************
缓冲池中的每一个有效数据帧, 存储GPS协议中要提取的信息, 
并且构成一个链表				
****************************************************************/
typedef struct DataFrame
{
    uint len;               //已经提取的数据长度
    uchar data[80];         //数据内容
    
    struct DataFrame* next; //指向下一个有效数据帧
}DataFrame;

/****************************************************************
缓冲池结构体, 控制缓冲池状态
****************************************************************/
typedef struct
{   
    uint num;               //有效数据帧个数
    DataFrame* head;        //数据帧头结点
    DataFrame* send;        //指向下一个待处理的数据帧
    DataFrame* rec;         //指向当前需要填充数据的数据帧
    
    uchar* AIM;             //目标提取信息关键词
    uchar* checkByte;       //与目标关键词匹配成功的当前位置
    uint mode;              //缓冲池模式, 0为检测模式(等待有效数据)
                            //1为接收模式, 存储每一个接收到的数据  
}Buf;

Buf buf;

/****************************************************************
存储提取到的zigbee结点地理数据, 每一个zigbee结点只有唯一的地理数据,
会根据接收到的GPS数据不断刷新, zigbee结点之间互相传递的也只有各个
结点的地理数据, 及该结构体.
****************************************************************/
typedef struct
{
    uchar time[9];           //实时时间
    uchar latitude[11];      //纬度
    uchar longitude[12];     //经度
    uchar qualityFactor;     //质量因子
    uchar satelliteNum[2];      //连接到的卫星数
}GeoInfo;

GeoInfo geoInfo;

/****************************************************************
从每一个有效数据帧中提取所需地理信息, 并且刷新geoInfo对象
****************************************************************/
void fetchInfo()
{
    uint i = 0, j = 0;
    uchar* ptr = (void*)&geoInfo;        //将整个结构体视为一个由字节构成的整体处理
    
    while(i <= sizeof(GeoInfo))
    {
        if(buf.send->data[j] == ',')
        {
            j++;
            continue;
        }
        
        ptr[i++] = buf.send->data[j++];
    }
    
}

/****************************************************************
串口0发送字符串函数				
****************************************************************/
void Uart0TX_Send_String(uchar* data, uint len)
{
    uint i;
    for(i = 0; i < len; i++)
    {
        U0DBUF = *data++;
        while(UTX0IF == 0);           //等待发送完成
        UTX0IF = 0;                   //置零发送完成标志
    } 
}

/****************************************************************
主函数							
****************************************************************/
int main(void)
{	
    init();
    
    buf.head = (DataFrame*)malloc(sizeof(DataFrame));
    memset(buf.head, 0, sizeof(DataFrame));
    buf.send = buf.rec = buf.head;
    buf.checkByte = buf.AIM = "$GPGGA";

    Uart0TX_Send_String("Link Start!\n", 12);
    
	while(1)
	{
        //限制只有至少存在两个有效数据帧时再处理, 防止数据提取速率超过接受速率, 破坏链表结构
        if(buf.num >= 2)  
        {
            fetchInfo();
            
            Uart0TX_Send_String("\nConnect Success! ---- ", 23);
            Uart0TX_Send_String((void*)&geoInfo, sizeof(geoInfo));
            buf.num--;
            
            buf.send = buf.send->next;
            
            free(buf.head);
            buf.head = buf.send;   
        }
        
                 
	}
    
    return 0;
}
/****************************************************************
串口接收一个字符:一旦有数据从串口传至CC2530,则进入中断，将接收到的数据赋值给变量temp.
****************************************************************/
#pragma vector = URX1_VECTOR
//为对应的中断向量指向对应中断函数
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//清中断标志
        
    if(buf.mode == 0)
    {
        if(U1DBUF == *buf.checkByte)
        {
            buf.checkByte++;
        }
        else
        {
            buf.checkByte = buf.AIM;
        }
        
        if(buf.checkByte == buf.AIM + 6)
        {
           buf.mode = 1;
        }
    }
    else
    {
        if(U1DBUF == '$')
        {
            if(buf.rec->len >= 67)
            {
                buf.rec->next = (DataFrame*)malloc(sizeof(DataFrame));
                buf.rec = buf.rec->next;
                memset(buf.rec, 0, sizeof(DataFrame));
                buf.num++;
            }
            else
            {
                Uart0TX_Send_String("Connect Fail! ---- ", 19);
                Uart0TX_Send_String(buf.rec->data, buf.rec->len);
                buf.rec->len = 0;
            }
            
            buf.mode = 0;
        }
        else
        {
            buf.rec->data[buf.rec->len++] = U1DBUF;        
        }
    }
   
 }
