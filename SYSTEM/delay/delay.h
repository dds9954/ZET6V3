#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/

	 
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_s(u8 s);

#endif





























