#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"
#include "ADS1292.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/   
//外部中断服务程序
void EXTIX_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	EXTI_ClearITPendingBit(EXTI_Line6);//清除中断标志
	//GPIOA.0	  中断线以及中断初始化配置 上升沿触发 PA0  WK_UP
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource6); 

	EXTI_InitStructure.EXTI_Line=EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	 //设置为中断请求，非事件请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;						 //外部中断使能
	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			 //使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				 //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 

}

//外部中断0服务程序 
void EXTI9_5_IRQHandler(void)
{
	if(EXTI->IMR&EXTI_Line6 && ADS_DRDY==0)//数据接收中断				
	{		
		EXTI_ClearITPendingBit(EXTI_Line6); //清除LINE0上的中断标志位  
		ADS1292_Read_Data(ads1292_Cache);//数据存到9字节缓冲区
		ads1292_recive_flag=1;
	}	
}
 

 
