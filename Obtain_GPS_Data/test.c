#include <iocc2530.h>
#include <stdlib.h>
#include <string.h>

#include "Init.h"

#define uint unsigned int
#define uchar unsigned char

/****************************************************************
������е�ÿһ����Ч����֡, �洢GPSЭ����Ҫ��ȡ����Ϣ, 
���ҹ���һ������				
****************************************************************/
typedef struct DataFrame
{
    uint len;               //�Ѿ���ȡ�����ݳ���
    uchar data[80];         //��������
    
    struct DataFrame* next; //ָ����һ����Ч����֡
}DataFrame;

/****************************************************************
����ؽṹ��, ���ƻ����״̬
****************************************************************/
typedef struct
{   
    uint num;               //��Ч����֡����
    DataFrame* head;        //����֡ͷ���
    DataFrame* send;        //ָ����һ�������������֡
    DataFrame* rec;         //ָ��ǰ��Ҫ������ݵ�����֡
    
    uchar* AIM;             //Ŀ����ȡ��Ϣ�ؼ���
    uchar* checkByte;       //��Ŀ��ؼ���ƥ��ɹ��ĵ�ǰλ��
    uint mode;              //�����ģʽ, 0Ϊ���ģʽ(�ȴ���Ч����)
                            //1Ϊ����ģʽ, �洢ÿһ�����յ�������  
}Buf;

Buf buf;

/****************************************************************
�洢��ȡ����zigbee����������, ÿһ��zigbee���ֻ��Ψһ�ĵ�������,
����ݽ��յ���GPS���ݲ���ˢ��, zigbee���֮�以�ഫ�ݵ�Ҳֻ�и���
���ĵ�������, ���ýṹ��.
****************************************************************/
typedef struct
{
    uchar time[9];           //ʵʱʱ��
    uchar latitude[11];      //γ��
    uchar longitude[12];     //����
    uchar qualityFactor;     //��������
    uchar satelliteNum[2];      //���ӵ���������
}GeoInfo;

GeoInfo geoInfo;

/****************************************************************
��ÿһ����Ч����֡����ȡ���������Ϣ, ����ˢ��geoInfo����
****************************************************************/
void fetchInfo()
{
    uint i = 0, j = 0;
    uchar* ptr = (void*)&geoInfo;        //�������ṹ����Ϊһ�����ֽڹ��ɵ����崦��
    
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
����0�����ַ�������				
****************************************************************/
void Uart0TX_Send_String(uchar* data, uint len)
{
    uint i;
    for(i = 0; i < len; i++)
    {
        U0DBUF = *data++;
        while(UTX0IF == 0);           //�ȴ��������
        UTX0IF = 0;                   //���㷢����ɱ�־
    } 
}

/****************************************************************
������							
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
        //����ֻ�����ٴ���������Ч����֡ʱ�ٴ���, ��ֹ������ȡ���ʳ�����������, �ƻ�����ṹ
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
���ڽ���һ���ַ�:һ�������ݴӴ��ڴ���CC2530,������жϣ������յ������ݸ�ֵ������temp.
****************************************************************/
#pragma vector = URX1_VECTOR
//Ϊ��Ӧ���ж�����ָ���Ӧ�жϺ���
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//���жϱ�־
        
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
