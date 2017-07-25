#include <iocc2530.h>
#include <string.h>
#define uint unsigned int
#define uchar unsigned char

//������ƵƵĶ˿�
#define LED1 P1_0
#define LED2 P1_1
#define LED3 P0_4

//UART���ܷ��ͻ�����(����ѭ������)
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
���ݻ�������ʼ��		
****************************************************************/
void dataBufInit(Data* data)
{
    memset(data->buf, 0, sizeof(data->buf));
    data->send = data->buf;
    data->rec = data->buf;
    data->len = 0;
}

/****************************************************************
����1�����ַ�������				
****************************************************************/
void Uart0TX_Send_String(uchar* Data, uint len)
{
    uint i;
    for(i = 0; i < len; i++)
    {
        U0DBUF = *Data++;
        while(UTX0IF == 0);           //�ȴ��������
        UTX0IF = 0;                   //���㷢����ɱ�־
    } 
}



/****************************************************************
��ʼ������0����					
****************************************************************/
void initUART0(void)
{
    CLKCONCMD &= ~0x40;                         //����ϵͳʱ��ԴΪ32MHZ����
    while(CLKCONSTA & 0x40);                    //�ȴ������ȶ�
    CLKCONCMD &= ~0x47;                         //����ϵͳ��ʱ��Ƶ��Ϊ32MHZ
   
    PERCFG = 0x00;				//UART0����λ��1 P0�� 2:RX 3:TX
    P0SEL = 0x0C;				//P0��������
    P2DIR &= ~0XC0;                             //P0������ΪUART0, �ڴ�������UART1��
                                                //������4,5����    
    
    U0CSR |= 0x80;				//��������ΪUART��ʽ
    U0GCR |= 8;				
    U0BAUD |= 59;				//��������Ϊ9600
    
    UTX0IF = 0;                                 //UART0 TX�жϱ�־��ʼ��λ0
                                                //���ڲ���Ҫ�����жϺ���, ����Ҫ������Ӧ
                                                //���ж�, �����ն˱�־��ʼ�ն��������
}

/****************************************************************
��ʼ������1����					
****************************************************************/
void initUART1(void)
{
   
    PERCFG = 0x00;				//UART1����λ��1 P0�� 4:TX 5:RX 
    P0SEL |= 0x30;				//P0�������� 
    
    U1CSR |= 0x80;				//��������ΪUART��ʽ
    U1GCR |= 8;				
    U1BAUD |= 59;				//��������Ϊ9600 
    
    U1CSR |= 0X40;				//�������

    URX1IF = 0;                                 //UART1 RX�жϱ�־��ʼ��λ0
    IEN0 |= 0x88;				//�����жϣ�U1�����ж�  
}

void UART0TX_Send_All()
{
    uint tempLen = data.len;  //���ǵ��ж���ʱ�����, ����ҪҪ
                              //��֤len�����ڷ���ʱ����, ��Ҫ���ȴ洢����
    if(data.send + tempLen < data.buf + DATA_BUF_LEN)
    {
        Uart0TX_Send_String(data.send, tempLen);
                
        data.send += tempLen;  //����ָ��ʼ��ָ����һ����Ҫ�������ݵ��׵�ַ
    }
    else
    {
        //�����������صĳ�����Ҫ��ַ���
        Uart0TX_Send_String(data.send, data.buf + DATA_BUF_LEN - data.send);
        Uart0TX_Send_String(data.buf, tempLen - (data.buf + DATA_BUF_LEN - data.send));
        data.send = data.buf + tempLen - (data.buf + DATA_BUF_LEN - data.send);
    }
            
    data.len -= tempLen; //��ȥ�Ѿ�������ɵĳ��� 
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
������							
****************************************************************/
void main(void)
{	
	P1DIR = 0x03; 				//P1����LED
	LED1 = 1;
	LED2 = 1;				//��LED
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
���ڽ���һ���ַ�:һ�������ݴӴ��ڴ���CC2530,������жϣ������յ������ݸ�ֵ������temp.
****************************************************************/
#pragma vector = URX1_VECTOR
//Ϊ��Ӧ���ж�����ָ���Ӧ�жϺ���
 __interrupt void UART1_ISR(void)
 {
 	URX1IF = 0;				//���жϱ�־
        
	*data.rec++ = U1DBUF;
        data.len++;
        
        //����ָ�����ڴ������βѭ��
        if(data.rec == data.buf + DATA_BUF_LEN)
        {
            data.rec = data.buf;
        }
 }
