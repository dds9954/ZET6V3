#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/
								    
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_5;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC

}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY_UP==0||KEY_LEFT==0||KEY_RIGHT==0||KEY_DOWN==0||KEY_SURE==0||KEY_CANCEL==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY_UP==0)return KEY_UP_PRES;
		else if(KEY_LEFT==0)return KEY_LEFT_PRES;
		else if(KEY_RIGHT==0)return KEY_RIGHT_PRES;
		else if(KEY_DOWN==0)return KEY_DOWN_PRES;
		else if(KEY_SURE==0)return KEY_SURE_PRES;
		else if(KEY_CANCEL==0)return KEY_CANCEL_PRES;
		
	}else if(KEY_UP==1&&KEY_LEFT==1&&KEY_RIGHT==1&&KEY_DOWN==1&&KEY_SURE==1&&KEY_CANCEL==1)key_up=1; 	    
 	return 0;// 无按键按下
}
