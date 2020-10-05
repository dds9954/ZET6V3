#ifndef __UART4_H
#define __UART4_H
#include "stdio.h"	
#include "sys.h" 
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/
 

#define UART4_REC_LEN  			200  	//定义最大接收字节数 200
#define UART4_SEND_LEN  			200  	//定义最大发送字节数 200

#define EN_UART4_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  UART4_RX_BUF[UART4_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8  UART4_TX_BUF[UART4_SEND_LEN]; 			//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern u16 UART4_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart4_init(u32 bound);
void u4_printf(char* fmt,...);  
#endif


