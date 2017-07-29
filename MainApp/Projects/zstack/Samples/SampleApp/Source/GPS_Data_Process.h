#ifndef GPS_DATA_PROCESS
#define GPS_DATA_PROCESS

#include "hal_types.h"

/****************************************************************
缓冲池中的每一个有效数据帧, 存储GPS协议中要提取的信息, 
并且构成一个链表				
****************************************************************/
typedef struct DataFrame
{
    uint16 len;               //已经提取的数据长度
    uint8 data[80];         //数据内容
    
    struct DataFrame* next; //指向下一个有效数据帧
}DataFrame;

/****************************************************************
缓冲池结构体, 控制缓冲池状态
****************************************************************/
typedef struct
{   
    uint16 num;               //有效数据帧个数
    DataFrame* head;        //数据帧头结点
    DataFrame* send;        //指向下一个待处理的数据帧
    DataFrame* rec;         //指向当前需要填充数据的数据帧
    
    uint8* AIM;             //目标提取信息关键词
    uint8* checkByte;       //与目标关键词匹配成功的当前位置
    uint16 mode;              //缓冲池模式, 0为检测模式(等待有效数据)
                            //1为接收模式, 存储每一个接收到的数据  
}GeoBuf;

/****************************************************************
存储提取到的zigbee结点地理数据, 每一个zigbee结点只有唯一的地理数据,
会根据接收到的GPS数据不断刷新, zigbee结点之间互相传递的也只有各个
结点的地理数据, 及该结构体.
****************************************************************/
#define GEO_INFO_NUM 7

uint8* GeoInfoName[GEO_INFO_NUM] = 
{
    "time=",
    "latitude=",
    "la_dir=",
    "longitude=",
    "lo_dir=",
    "qualityFactor=",
    "satelliteNum=",
};

//extern uint8 geoInfoData[GEO_INFO_NUM][20];

typedef struct
{
    uint8 time[9];           //实时时间
    uint8 latitude[10];      //纬度
    uint8 la_dir[1];
    uint8 longitude[11];     //经度
    uint8 lo_dir[1];
    uint8 qualityFactor[1];     //质量因子
    uint8 satelliteNum[2];      //连接到的卫星数
}GeoInfo;

const uint16 geoInfoLen[GEO_INFO_NUM + 1] = 
{
    0, 9, 19, 20, 31, 32, 33, 35
};

#endif