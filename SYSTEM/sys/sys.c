#include "sys.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
