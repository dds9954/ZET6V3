#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/

u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

 			
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   			
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   

void SPI1_Init(void);			 //初始化SPI口
void SPI2_Init(void);			 //初始化SPI口	


void OLED_SPI_Init(void);



#endif


