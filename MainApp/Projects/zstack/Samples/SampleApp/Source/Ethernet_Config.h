#ifndef ETHERNET_CONFIG
#define ETHERNET_CONFIG

#include "OSAL.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "hal_led.h"
#include "AF.h"

char* COMMANDS[] =
{
    "AT+RST\r\n",       //重启ESP8266
    "AT+CWMODE=3\r\n",    //切换ESP8266工作模式, 参数 - (1:Station | 2:AP | 3:Station+AP)
    "AT+CIPMUX=0\r\n",    //设置是否为多连接模式, 参数 - (0:单连接 | 1:多连接)
    "AT+CIPMODE=0\r\n",   //选择传输模式, 参数 - (0:非透传模式 | 1:透传模式)
    "AT+CWJAP=\"P8MAX\",\"574288475\"\r\n",     //加入其它AP结点的网络中, 参数 - ("网络名"),("密码")
    "AT+CIPSTART=\"UDP\",\"192.168.43.224\",50000\r\n",  //建立TCP或UDP连接, 参数 - ("类型"),("目标IP地址"),(端口号)
    "AT+CIPSEND",   //发送数据包, 设置读取的数据包长度, 超长则会被截取, 只会读取一次, 参数 - (长度)
    "AT+CIPCLOSE\r\n",  //关闭TCP或UDP连接, 参数 - (连接ID号1~5, 多连接模式下5为全部关闭)
    "AT+CIPSERVER\r\n", //设置ESP8266为服务器模式, 需要在多连接状态下, 参数 - (0:关闭 | 1:开启),(端口号, 缺省为333)
    "AT+CIPSTO\r\n",    //设置服务器超时时间, 参数 - (时间0~28800s)
    "AT+CISTATUS\r\n",  //获得当前模块连接状态, 参数 - 无
    "AT+CIFSR\r\n",     //获取本机的IP地址, 参数 - 无
};

void initESP(void);
void uploadData(afIncomingMSGPacket_t *pkt);

void initESP(void)
{
    uint8 stage = 5;

    for(; stage < 6; stage++)
    {
        HalUARTWrite(0, COMMANDS[stage], osal_strlen(COMMANDS[stage]));
        if(stage == 3)
        {
            HalUARTWrite(0, COMMANDS[0], osal_strlen(COMMANDS[0]));   //重启
        }
    }

    HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);   //未入网前为关闭状态, 入网后长亮
}

void uploadData(afIncomingMSGPacket_t *pkt)
{
    uint8 len[20];
    _ltoa(pkt->cmd.DataLength, len, 10);
    HalUARTWrite(0, COMMANDS[6], osal_strlen(COMMANDS[6]));
    HalUARTWrite(0, "=", 1);
    HalUARTWrite(0, len, osal_strlen(len));
    HalUARTWrite(0, "\r\n", 2);
    HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);
    
    HalLedBlink(HAL_LED_2, 2, 50, 500); // LED2亮 2次, 暗的时间占闪烁周期的50%, 周期为500毫秒
}

#endif