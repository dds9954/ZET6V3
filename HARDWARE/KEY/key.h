#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"	 
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/


#define KEY_UP      GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//读取上按键
#define KEY_LEFT    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//读取左按键
#define KEY_RIGHT   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)//读取右按键
#define KEY_DOWN    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//读取下按键
#define KEY_SURE    GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)//读取确认按键
#define KEY_CANCEL  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)//读取取消按键

#define KEY_UP_PRES 	1	//KEY_UP按下
#define KEY_DOWN_PRES	2	//KEY_LEFT按下
#define KEY_LEFT_PRES   3	//KEY_RIGHT按下
#define KEY_RIGHT_PRES 	4	//KEY_DOWN按下
#define KEY_SURE_PRES	5	//KEY_SURE按下
#define KEY_CANCEL_PRES	6	//KEY_CANCEL按下

void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数					    
#endif
