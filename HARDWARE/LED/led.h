#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/

#define LED_R PCout(0)// PB5
#define LED_G PCout(1)// PE5	
#define LED_B PCout(2)// PE5

void LED_Init(void);//初始化

		 				    
#endif
