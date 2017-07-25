#include <iocc2530.h>
#include <string.h>
#define uint unsigned int
#define uchar unsigned char

//定义控制灯的端口
#define LED1 P1_0
#define LED2 P1_1
#define LED3 P0_4

//UART接受发送缓存区(单向循环队列)
#define DATA_BUF_LEN 1000
#define DATA_PART_LEN 73
typedef struct
{
    uchar buf[DATA_BUF_LEN];
    uchar* send;
    uchar* rec;
    uint len;
    
    uchar part[DATA_PART_LEN];
    uchar* AIM;
}Data;

Data data = {{0}, data.buf, data.buf, 0, {0}, "GPGGA"};

/****************************************************************
数据缓存区初始化		
****************************************************************/
void dataBufInit(Data* data)
{
    memset(data->buf, 0, sizeof(data->buf));
    data->send = data->buf;
    data->rec = data->buf;
    data->len = 0;
}

/****************************************************************
串口1发送字符串函数				
****************************************************************/
void Uart0TX_Send_String(uchar* Data, uint len)
{
    uint i;
    for(i = 0; i < len; i++)
    {
        U0DBUF = *Data++;
        while(UTX0IF == 0);           //等待发送完成
        UTX0IF = 0;                   //置零发送完成标志
    } 
}



/****************************************************************
初始化串口0函数					
****************************************************************/
void initUART0(void)
{
    CLKCONCMD &= ~0x40;                         //设置系统时钟源为32MHZ晶振
    while(CLKCONSTA & 0x40);                    //等待晶振稳定
    CLKCONCMD &= ~0x47;                         //设置系统主时钟频率为32MHZ
   
    PERCFG = 0x00;				//UART0备用位置1 P0口 2:RX 3:TX
    P0SEL = 0x0C;				//P0用作串口
    P2DIR &= ~0XC0;                             //P0优先作为UART0, 在此设置下UART1会
                                                //优先用4,5引脚    
    
    U0CSR |= 0x80;				//串口设置为UART方式
    U0GCR |= 8;				
    U0BAUD |= 59;				//波特率设为9600
    
    UTX0IF = 0;                                 //UART0 TX中断标志初始置位0
                                                //由于不需要调用中断函数, 不需要开启对应
                                                //的中断, 但是终端标志是始终都会产生的
}

/****************************************************************
初始化串口1函数					
****************************************************************/
void initUART1(void)
{
   
    PERCFG = 0x00;				//UART1备用位置1 P0口 4:TX 5:RX 
    P0SEL |= 0x30;				//P0用作串口 
    
    U1CSR |= 0x80;				//串口设置为UART方式
    U1GCR |= 8;				
    U1BAUD |= 59;				//波特率设为9600 
    
    U1CSR |= 0X40;				//允许接收

    URX1IF = 0;                                 //UART1 RX中断标志初始置位0
    IEN0 |= 0x88;				//开总中断，U1接收中断  
}

void UART0TX_Send_All()
{
    uint tempLen = data.len;  //考虑到中断随时会产生, 必须要要
                              //保证len参数在发送时不变, 就要事先存储下来
    if(data.send + tempLen < data.buf + DATA_BUF_LEN)
    {
        Uart0TX_Send_String(data.send, tempLen);
                
        data.send += tempLen;  //发送指针始终指向下一次需要发送数据的首地址
    }
    else
    {
        //如果超出缓存池的长度则要拆分发送
        Uart0TX_Send_String(data.send, data.buf + DATA_BUF_LEN - data.send);
        Uart0TX_Send_String(data.buf, tempLen - (data.buf + DATA_BUF_LEN - data.send));
        data.send = data.buf + tempLen - (data.buf + DATA_BUF_LEN - data.send);
    }
            
    data.len -= tempLen; //减去已经发送完成的长度 
}

void UART0TX_Send_Part()
{
    uint j;
    while(data.len--)
    {
        if(*data.send++ == '$')
        {
            for(j = 0; j < DATA_PART_LEN; j++)
            {
                data.part[j] = *data.send++;
                data.len--;
                if(data.send == data.buf + DATA_BUF_LEN)
                {
                    data.send = data.buf;
                }
                
                if(j == 4 && strncmp(data.part, data.AIM, 5) != 0)
                {
                    break;
                }
            }
            
            if(j == DATA_PART_LEN)
            {
                Uart0TX_Send_String(data.part, j);
            }        
        }
        
        if(data.send == data.buf + DATA_BUF_LEN)
        {
            data.send = data.buf;
        }
    }
}
/****************************************************************
主函数							
****************************************************************/
void main(void)
{	
	P1DIR = 0x03; 				//P1控制LED
	LED1 = 1;
	LED2 = 1;				//关LED
        LED3 = 1;
        
	initUART0();
        initUART1();
        //dataBufInit(&data);
        
        Uart0TX_Send_String("Link Start!\n", 12);
        
	while(1)
	{
            //UART0TX_Send_All();

            UART0TX_Send_Part();
            
            
	}
}
/****************************************************************
串口接收一个字符:一旦有数据从串口传至CC2530,则进入中断，将接收到的数据赋值给变量temp.
****************************************************************/
#pragma vector = URX1_VECTOR
//为对应的中断向量指向对应中断函数
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//清中断标志
        
	*data.rec++ = U1DBUF;
        data.len++;
        
        //接收指针在内存池中首尾循环
        if(data.rec == data.buf + DATA_BUF_LEN)
        {
            data.rec = data.buf;
        }
 }
