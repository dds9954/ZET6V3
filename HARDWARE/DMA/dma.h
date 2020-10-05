#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/


void DMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar);
void DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u16 len);
	   
#endif




