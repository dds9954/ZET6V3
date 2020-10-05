#include "sys.h"
#include "stm32f10x_i2c.h"
#include "ads115.h"
/*--------------------------------------------------------

2020电子设计竞赛-TI杯  综合测试程序
说明：本程序只是2020 TI杯所要用到的传感器功能测试程序。
作者：李孟然
版本：V1.1	 

---------------------------------------------------------*/
static void ADS1115_delay(u16 D)
{
	while(--D);
}

void delay_nms(u16 ms)
{
	u16 i;
	u32 M = 0;//720W
	for(i = 0;i < ms; i++)
	for(M=12000;M > 0;M--);
}

void delay_nus(u16 us)
{
	u16 i;
	u16 M = 0;//720W
	for(i = 0;i < us; i++)
	for(M=72;M > 0;M--);
}



/////////////////PA8 SDA////PA9 SCL///////////////////////////////////
void ADS1115_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC ,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   //D2:SCL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  //C12:SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    SDA_A1;
    SCL_A1;
	delay_nms(5);

}

//I2C总线启动
void I2CStart_A(void)
{
   SDA_A1;
   ADS1115_delay(5);
   SCL_A1;
   ADS1115_delay(5);
   SDA_A0;
   ADS1115_delay(5);//MIN 160ns
   SCL_A0;
   ADS1115_delay(5);
}




//I2C停止总线
void I2CStop_A(void)
{
   SDA_A0;
   ADS1115_delay(5);
   SCL_A1;
   ADS1115_delay(5);
   SDA_A1;
   ADS1115_delay(5);//MIN 160ns
}



//I2C 写一字节
void I2CWriteByte_A(u8 DATA)
{
      u8 i;
      SCL_A0;
      for(i = 0;i < 8; i++)
      {
         if(DATA&0x80)
         {
              SDA_A1;
         }
         else
         {
              SDA_A0;
         }
         SCL_A1;//按照手册不需延时
         ADS1115_delay(5);
         SCL_A0;
         ADS1115_delay(5);
         DATA	= DATA << 1;  
      }
      SDA_A1;
      SCL_A1;
      ADS1115_delay(5);
      SCL_A0;
}


//I2C 读一字节
u8 I2CReadByte_A(void)
{
    u8 TData=0,i;
    for(i=0;i<8;i++)
    {
		SCL_A1;
		ADS1115_delay(5);
		TData=TData<<1;
		if(SDA_AI)
		{
			TData|=0x01;
		}
		SCL_A0;
		ADS1115_delay(5);
    }
    SCL_A0;
    ADS1115_delay(5);
    SDA_A0;
    ADS1115_delay(5);
    SCL_A1;
    ADS1115_delay(5);
    SCL_A0;
    ADS1115_delay(5);
    SDA_A1;
    return TData;
}



/*********************************************************************
*函数名称:  ADS1115Config
*描	   述： 设置ADS1115包括通道配置，采样时间等等
*参	   数： HCMD ：命令字高8位(通道，量程，转换模式)
			LCMD : 命令字低8位(采样率设置 比较模式 有效电平 信号输出锁存)
*返	   回； 无
********************************************************************/
void ADS1115Config_A(u8 LCMD,u8 HCMD)
{
    u8 i=0;
    u8 Initdata[4];

    Initdata[0] = WR_REG;  // 地址0x90  器件ADR接地 写寄存器
    Initdata[1] = CONF_REG;// 配置寄存器
    Initdata[2] = HCMD;    // 配置字高字节
    Initdata[3] = LCMD;    // 配置字低字节
    SCL_A1;
    I2CStart_A();        //开启
    for(i=0;i<4;i++)
    {
    	I2CWriteByte_A(Initdata[i]);
    	ADS1115_delay(10);
    }
    I2CStop_A();         //关闭
}

void SetThresHold_A(u16 L_TH,u16 H_TH)        //高低阀门设置
{
   SCL_A1;
   I2CStart_A();      // 开启
   I2CWriteByte_A(WR_REG);
   I2CWriteByte_A(LOTH_REG);//最低阀值寄存器
   I2CWriteByte_A((L_TH>>8));
   I2CWriteByte_A(L_TH);
   I2CStop_A();       //关闭

   I2CStart_A();     //开启
   I2CWriteByte_A(WR_REG);
   I2CWriteByte_A(HITH_REG);//最高阀值寄存器
   I2CWriteByte_A((H_TH>>8));
   I2CWriteByte_A(H_TH);
   I2CStop_A();      //关闭
}


/*******************************************************************
*函数名称:  ReadAD_A
*描	   述： 获取AD转换的值
*参	   数： 获取的值为在前面设置的那个通道
*返	   回； 无
********************************************************************/
u16 ReadAD_A(void)
{
   u16 Data[2]={0,0};

   SCL_A1;
   I2CStart_A();
   I2CWriteByte_A(WR_REG);
   I2CWriteByte_A(DATA_REG);
   I2CStop_A();
   
   I2CStart_A();
   I2CWriteByte_A(RE_REG);
   Data[0] = I2CReadByte_A();
   Data[1] = I2CReadByte_A();
   I2CStop_A();
   
   Data[0] = Data[0]<<8 | Data[1];
   return  (Data[0]);//&0x7fff
}

u16 getad(u8 LCMD,u8 HCMD)
{
   float value=0;
 //   ADS1115_Init();  
  //  delay_nms(5);
    ADS1115Config_A(LCMD,HCMD);		  //AINP = AIN0 and AINN = AIN1 (default)
  //  delay_nms(5); 
    value=ReadAD_A();
    return value;
}

u16 lvbo(u8 LCMD,u8 HCMD)        //求平均值
{
	int k;
	float U=0,temp;
	
	 
	for(k=0;k<500;k++)
	{
		U+=getad(LCMD,HCMD);	
	}
	temp=U;
		U=0;
	return (temp/500);	
}






