/**********************************************************
                       ¿µÍþµç×Ó
¹¦ÄÜ£ºstm32f103rct6¿ØÖÆ£¬20MHzÊ±ÖÓ£¬ AD9954ÕýÏÒ²¨µãÆµÊä³ö£¬·¶Î§0-130M(¿É±à¼­0-200M)£¬
			É¨ÆµÄ¬ÈÏÕýÏÒ²¨ ¡£
			ÏÔÊ¾£º12864cog
½Ó¿Ú£º¿ØÖÆ½Ó¿ÚÇë²ÎÕÕAD9954.h  °´¼ü½Ó¿ÚÇë²ÎÕÕkey.h
Ê±¼ä£º2015/11/10
°æ±¾£º1.0
×÷Õß£º¿µÍþµç×Ó
ÆäËû£º

¸ü¶àµç×ÓÐèÇó£¬Çëµ½ÌÔ±¦µê£¬¿µÍþµç×Ó½ß³ÏÎªÄú·þÎñ ^_^
https://shop110336474.taobao.com/?spm=a230r.7195193.1997079397.2.Ic3MRJ

**********************************************************/
#include "AD9954.h"
#include "delay.h"

//ÏµÍ³ÆµÂÊfosc£¨Íâ²¿¾§ÕñÆµÂÊ£©£¬ÏµÍ³ÆµÂÊ=fosc*M
#define fosc  20                        //¾§ÕñÆµÂÊ
#define PLL_MULTIPLIER  19              //PLL±¶ÆµÊý£¨4--20£©
#define fs  (fosc*PLL_MULTIPLIER)       //ÏµÍ³Ê±ÖÓÆµÂÊ

//double fH_Num=11.2204;
//double fH_Num=10.73741824;
//double fH_Num=11.3671588397205;//
double fH_Num = 11.3025455157895;
/*
** º¯ÊýÃû³Æ £ºvoid GPIO_AD9954_Init(void)
** º¯Êý¹¦ÄÜ £ºAD9954½Ó¿ÚIO³õÊ¼»¯
**            AD9954_CS----------PB9	 		OUT
**	      		AD9954_SCLK--------PB10	 		OUT
**            AD9954_SDIO--------PB11 	  OUT
**            AD9954_OSK---------PB12   	OUT
**          	PS0----------------PB13		  OUT
**	      		PS1----------------PB14	 		OUT
**            IOUPDATE-----------PB15	  	OUT

**	      		AD9954_SDO---------PB5	  	IN
**            AD9954_IOSY--------PB6	  	OUT
**            AD9954_RES---------PB7    	OUT
**            AD9954_PWR---------PB8    	OUT
*/
//void GPIO_AD9954_Init(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE);	 //Ê¹ÄÜPB,PE¶Ë¿ÚÊ±ÖÓ

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
//	GPIO_Init(GPIOC, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_7|GPIO_Pin_6;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
//	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_1|GPIO_Pin_0;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	    		 //LED1-->PE.5 ¶Ë¿ÚÅäÖÃ, ÍÆÍìÊä³ö
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //ÉèÖÃ³ÉÉÏÀ­ÊäÈë
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//³õÊ¼»¯GPIOE2,3,4

//	AD9954_IOSY=0;
//	AD9954_OSK=0;
//	AD9954_PWR=0;
//}
void GPIO_AD9954_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG, ENABLE);	 //Ê¹ÄÜPB,PE¶Ë¿ÚÊ±ÖÓ
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_12|GPIO_Pin_14|GPIO_Pin_15;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
 GPIO_Init(GPIOF, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
 GPIO_Init(GPIOG, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5
	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_13|GPIO_Pin_2;				 //LED0-->PB.5 ¶Ë¿ÚÅäÖÃ
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
// GPIO_Init(GPIOC, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOB.5	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		 //LED1-->PE.5 ¶Ë¿ÚÅäÖÃ, ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //ÉèÖÃ³ÉÉÏÀ­ÊäÈë
 	GPIO_Init(GPIOF, &GPIO_InitStructure);//³õÊ¼»¯GPIOE2,3,4
	
	AD9954_IOSY=0;
	AD9954_OSK=0;
	AD9954_PWR=0;
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid AD9954_RESET(void)
** º¯Êý¹¦ÄÜ £º¸´Î»AD9954
** º¯ÊýËµÃ÷ £º²»¸´Î»Ò²¿ÉÒÔ
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void AD9954_RESET(void)
{
	AD9954_CS = 1;
	AD9954_RES = 0;
	delay_ms(10);
	AD9954_RES = 1;
	delay_ms(10);
	AD9954_RES = 0;
	delay_ms(10);
	AD9954_CS = 0;
	AD9954_SCLK = 0;
	PS0 = 0;
	PS1 = 0;
	IOUPDATE = 0;
	AD9954_CS = 1;
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid AD9954_SENDBYTE(u8 dat)
** º¯Êý¹¦ÄÜ £ºÍùAD9954·¢ËÍÒ»¸ö×Ö½ÚµÄÄÚÈÝ
** º¯ÊýËµÃ÷ £ºAD9954µÄ´«ÊäËÙ¶È×î´óÎª25M£¬ËùÒÔ²»¼ÓÑÓÊ±Ò²¿ÉÒÔ
** Èë¿Ú²ÎÊý £º´ý·¢ËÍ×Ö½Ú
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void AD9954_SENDBYTE(u8 dat)
{
	u8 i;
	for (i = 0;i<8;i++)
	{
		AD9954_SCLK = 0;
		if (dat & 0x80)
		{
			AD9954_SDIO = 1;
		}
		else
		{
			AD9954_SDIO = 0;
		}
		AD9954_SCLK = 1;
		dat <<= 1;
	}
}
u8 AD9954_ReadByte(void)
{
	u8 i,dat=0;
	for (i = 0;i<8;i++)
	{
		AD9954_SCLK = 0;
		dat|=AD9954_SDO;
		AD9954_SCLK = 1;
		dat <<= 1;
	}
	return dat;
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid UPDATE(void)
** º¯Êý¹¦ÄÜ £º²úÉúÒ»¸ö¸üÐÂÐÅºÅ£¬¸üÐÂAD9954ÄÚ²¿¼Ä´æÆ÷£¬
** º¯ÊýËµÃ÷ £º¿ÉÒÔ²»¼ÓÈÎºÎÑÓÊ±
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void UPDATE(void)
{
	IOUPDATE=0;
//	delay_us(100); 
	IOUPDATE = 1;
	delay_us(2);
	IOUPDATE = 0;
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid AD9954_Init(void))
** º¯Êý¹¦ÄÜ £º³õÊ¼»¯AD9954µÄ¹Ü½ÅºÍ×î¼òµ¥µÄÄÚ²¿¼Ä´æÆ÷µÄÅäÖÃ£¬
** º¯ÊýËµÃ÷ £ºÄ¬ÈÏ¹Øµô±È½ÏÆ÷µÄµçÔ´£¬Òò°åÉÏµÄ¾§ÕñÎª20MHz£¬×î´ó²ÉÓÃÁË19±¶Æµ£¬Îª380M
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/


//¶Á¼Ä´æÆ÷
u32 Read_Vau1(void)
{
    uint32_t ret;
//    char i;
    AD9954_CS=1;
    AD9954_SENDBYTE(CFR1);
    ret = AD9954_ReadByte();
    ret<<=8;
    ret += AD9954_ReadByte();
    ret<<=8;
    ret += AD9954_ReadByte();
    ret<<=8;
    ret += AD9954_ReadByte();
    AD9954_CS=1;
    return ret;
}
u32 Read_Vau2(void)
{
    uint32_t ret;
//    char i;
    AD9954_CS=1;
    AD9954_SENDBYTE(CFR2);
    ret = AD9954_ReadByte();
    ret<<=8;
    ret += AD9954_ReadByte();
    ret<<=8;
    ret += AD9954_ReadByte();
    AD9954_CS=0;
    return ret;
}
void AD9954_Init(void)
{
	GPIO_AD9954_Init();
	AD9954_RESET();
	delay_ms(300);
	AD9954_CS = 0;
	//single tone
////	AD9954_SENDBYTE(CFR1);//µØÖ·0Ð´²Ù×÷
////	AD9954_SENDBYTE(0x02);//
////	AD9954_SENDBYTE(0x00);
////	AD9954_SENDBYTE(0x00);
////	AD9954_SENDBYTE(0x00);//±È½ÏÆ÷Ê¹ÄÜ	
	AD9954_SENDBYTE(CFR1);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0X00);//
	AD9954_SENDBYTE(0X00);
	AD9954_SENDBYTE(0X00);
	AD9954_SENDBYTE(0x00);//±È½ÏÆ÷Ê¹ÄÜ	

	AD9954_SENDBYTE(CFR2);//µØÖ·1Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x00);
#if fs>400
    #error "ÏµÍ³ÆµÂÊ³¬¹ýÐ¾Æ¬×î´óÖµ"
#endif
#if fs>=250
    AD9954_SENDBYTE(PLL_MULTIPLIER<<3|0x04|0X03);
#else
    AD9954_SENDBYTE(PLL_MULTIPLIER<<3);
#endif
//	AD9954_SENDBYTE(0x24);//4±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M
	AD9954_CS=1;
}
u32 Get_FTW(double Real_fH)
{
		return (u32)(fH_Num*Real_fH);
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid AD9954_SETFRE(float f)
** º¯Êý¹¦ÄÜ £ºÉèÖÃAD9954µ±Ç°µÄÆµÂÊÊä³ö£¬²ÉÓÃµÄÊÇµ¥Ò»ÆµÂÊÊä³ö
** º¯ÊýËµÃ÷ £ºÒòÎª²ÉÓÃµÄ¸¡µãÊý½øÐÐ¼ÆËã£¬×ª»»¹ý³ÌÖÐ»á³öÏÖÎó²î£¬Í¨¹ýµ÷Õû¿ÉÒÔ¾«È·µ½0.1HzÒÔÄÚ
** Èë¿Ú²ÎÊý £ºÓûÉèÖÃµÄÆµÂÊÖµ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void AD9954_SETFRE(double f)//single tone
{
	u32 date;
	AD9954_CS = 0;
	
	date = Get_FTW(f);//det=(f/fclk)x2^32=10.7374xf
	AD9954_SENDBYTE(FTW0);//FTW0µØÖ·
	AD9954_SENDBYTE((u8)(date >> 24));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)(date >> 16));
	AD9954_SENDBYTE((u8)(date >> 8));
	AD9954_SENDBYTE((u8)date);
	AD9954_CS=1;
	UPDATE();
//	fH_Num+=0.001;
}

void DirectSwitchPSK(float f, float phase1, float phase2, float phase3, float phase4)
{
	u16 date;
	AD9954_CS = 0;
	IOUPDATE = 0;
	delay_us(1);
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	//
	AD9954_SENDBYTE(0x01);//µØÖ·1Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0xA4);//4±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M
	AD9954_SETFRE(f);//ÔØ²¨ÆµÂÊ
	PS0 = 0;
	PS1 = 0;
	AD9954_SENDBYTE(0x07);
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);//final address:0x000
	AD9954_SENDBYTE(0x00);//start address:0x000;
	AD9954_SENDBYTE(0x00);//RAM0¹¤×÷ÓÚÄ£Ê½0,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î
	UPDATE();
	date = 45.51*phase1;
	date = date << 2;
	AD9954_SENDBYTE(0x0b);
	AD9954_SENDBYTE((u8)(date >> 8));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)date);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);

	PS0 = 1;//ram1
	PS1 = 0;

	AD9954_SENDBYTE(0x08);
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x01);//final address:0x0001
	AD9954_SENDBYTE(0x04);//start address:0x0001
	AD9954_SENDBYTE(0x00);//RAM1¹¤×÷ÓÚÄ£Ê½0,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î
	UPDATE();
	date = 45.51*phase2;
	date = date << 2;
	AD9954_SENDBYTE(0x0b);
	AD9954_SENDBYTE((u8)(date >> 8));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)date);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);

	PS0 = 0;//ram2
	PS1 = 1;

	AD9954_SENDBYTE(0x09);
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x02);//final address:0x0002
	AD9954_SENDBYTE(0x08);//start address:0x0002
	AD9954_SENDBYTE(0x00);
	UPDATE();
	date = 45.51*phase3;
	date = date << 2;
	AD9954_SENDBYTE(0x0b);
	AD9954_SENDBYTE((u8)(date >> 8));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)date);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	PS0 = 1;//ram3
	PS1 = 1;
	AD9954_SENDBYTE(0x0a);
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x03);//final address:0x0003
	AD9954_SENDBYTE(0x0c);//start address:0x0003
	AD9954_SENDBYTE(0x00);
	UPDATE();
	date = 45.51*phase4;
	date = date << 2;
	AD9954_SENDBYTE(0x0b);
	AD9954_SENDBYTE((u8)(date >> 8));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)date);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);

	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0xa0);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	AD9954_CS = 1;
	UPDATE();
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid Generate_PSK(float f,u16 *phase)
** º¯Êý¹¦ÄÜ £ºPSKµ÷ÖÆÐÅºÅÊä³ö
** º¯ÊýËµÃ÷ £ºÍ¨¹ýRAM¶ËÀ´¿ØÖÆÐÅºÅµÄÏàÎ»£¬´Ó¶øÊµÏÖPSKÐÅºÅµÄµ÷ÖÆ
**            ÕâÀïÊ¹ÓÃÒ»¸öRAM¶ÎµÄÑ­»·Ä£Ê½£¬PSKÐÅºÅÎª8ÎªÊý¾ÝÎ»¡£
**            RAMµÄ¸üÐÂËÙ¶ÈÓÉram rate¶¨Ê±¼Ä´æÆ÷¿ØÖÆ£¬¼Ä´æÆ÷Îª16Î»£¬ÊµÏÖ¶ÔÏµÍ³Ìá¹©µÄ100MÊ±ÖÓ1~65535·ÖÆµ£¬Ð´0ÊÇÎÞÐ§µÄ
**			      µ±¸úÊä³öPSKÔØ²¨µÄÆµÂÊÊÇram rate¸úÐÂÆµÂÊµÄÕûÊý±¶Ê±£¬²ÅÄÜ±£Ö¤Ã¿´ÎµÄÏàÎ»¸Ä±äÒ»ÖÂ
** Èë¿Ú²ÎÊý £ºfloat f   £ºPSKÔØ²¨µÄÆµÂÊ
** 			  u16 *phase: Ð´ÈëÏàÎ»ÀÛ¼ÓÆ÷µÄÏàÎ»Öµ£¬180ÊµÏÖ¶ÔÏàÎ»·­×ª£¬0²»¸Ä±äÏàÎ»£¬PSKÐÅºÅÎª8Î»£¬ÐèÎª8Î»µÄÊý×é
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void Generate_PSK(float f, u16 *phase)
{
	u8 i;
	u16 date;
	AD9954_CS = 0;
	IOUPDATE = 0;

	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	//
	AD9954_SENDBYTE(0x01);//µØÖ·1Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0xA4);//4±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M
	AD9954_SETFRE(f);//ÔØ²¨ÆµÂÊ

	AD9954_SENDBYTE(0x04);//FTW0µØÖ·
	AD9954_SENDBYTE(0x00);//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE(0x10);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	UPDATE();

	PS0 = 0;
	PS1 = 0;
	AD9954_SENDBYTE(0x07);
	AD9954_SENDBYTE(0x00);//ramp rate=0x0400
	AD9954_SENDBYTE(0x04);
	AD9954_SENDBYTE(0x07);//final address:0x007
	AD9954_SENDBYTE(0x00);//start address:0x000;
	AD9954_SENDBYTE(0x80);//RAM0¹¤×÷ÓÚÄ£Ê½4,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î
	UPDATE();

	AD9954_SENDBYTE(0x0b);
	for (i = 0;i<8;i++)
	{
		date = 45.51*phase[i];
		date = date << 2;
		AD9954_SENDBYTE((u8)(date >> 8));//ÆµÂÊ¿ØÖÆ×Ö
		AD9954_SENDBYTE((u8)date);
		AD9954_SENDBYTE(0x00);
		AD9954_SENDBYTE(0x00);
	}
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0xc0);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	AD9954_CS = 1;
	UPDATE();
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid Generate_FM(u32 *fre)
** º¯Êý¹¦ÄÜ £ºFMµ÷ÖÆÐÅºÅÊä³ö
** º¯ÊýËµÃ÷ £ºÍ¨¹ýRAM¶ËÀ´¿ØÖÆÐÅºÅµÄÆµÂÊ£¬´Ó¶øÊµÏÖFMÐÅºÅµÄµ÷ÖÆ
**            ÕâÀïÊ¹ÓÃÒ»¸öRAM¶ÎµÄÑ­»·Ä£Ê½£¬PSKÐÅºÅÎª8ÎªÊý¾ÝÎ»¡£
**            RAMµÄ¸üÐÂËÙ¶ÈÓÉram rate¶¨Ê±¼Ä´æÆ÷¿ØÖÆ£¬¼Ä´æÆ÷Îª16Î»£¬ÊµÏÖ¶ÔÏµÍ³Ìá¹©µÄ100MÊ±ÖÓ1~65535·ÖÆµ£¬Ð´0ÊÇÎÞÐ§µÄ
**			  ram rateµÄ¸úÐÂËÙ¶ÈÎªµ÷ÖÆÐÅºÅµÄÆµÂÊ£¬ÕâÀïÈç1KHz£¬Ð´Èëram rateµÄÖµÎª0X0C35,
** Èë¿Ú²ÎÊý £ºu32 *fre£ºFMÐÅºÅÆµÂÊÖµ£¬ÕâÀï²ÉÓÃµÄÊÇ32µãÆµÂÊ²ÉÑù£¬µ÷ÓÃ´Ëº¯ÊýÖ®Ç°ÐèÏÈÉèÖÃºÃÆµÂÊ±íµÄÖµ£¬Æä°´ÕýÏÒ¹æÂÉ¸Ä±ä
**            for(i=0;i<32;i++)
**				{
**					fre[i]=Fc+Fshift*sin(wt);
**				}
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void Generate_FM(u32 *fre)
{
	u8 i;
	u32 date;
	AD9954_CS = 0;
	IOUPDATE = 0;

	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);

	AD9954_SENDBYTE(0x01);//µØÖ·1Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x24);//4±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M

	PS0 = 0;
	PS1 = 0;
	AD9954_SENDBYTE(0x07);
	AD9954_SENDBYTE(0x35);//ramp rate=32kHz
	AD9954_SENDBYTE(0x0C);
	AD9954_SENDBYTE(0x1F);//final address:0x000
	AD9954_SENDBYTE(0x00);//start address:0x000;
	AD9954_SENDBYTE(0x80);//RAM0¹¤×÷ÓÚÄ£Ê½0,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î
	UPDATE();

	AD9954_SENDBYTE(0x0b);
	for (i = 0;i<32;i++)
	{
		date = 10.73741*fre[i];
		AD9954_SENDBYTE((u8)(date >> 24));//ÆµÂÊ¿ØÖÆ×Ö
		AD9954_SENDBYTE((u8)(date >> 16));
		AD9954_SENDBYTE((u8)(date >> 8));
		AD9954_SENDBYTE((u8)date);
	}
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x80);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	AD9954_CS = 1;
	UPDATE();
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid ASK_Init(void)
** º¯Êý¹¦ÄÜ £ºASKµ÷ÖÆÐÅºÅÊä³ö³õÊ¼»¯
** º¯ÊýËµÃ÷ £ºASKµ÷ÖÆ£¬ÐèÒªÏÈÉèÖÃºÃÔØ²¨ÆµÂÊ£¬È»ºó¸Ä±äDAC±ÈÀýÒò×Ó scale factorÆäÎª14Î»£¬Í¨¹ýÉèÖÃÎª×î´óºÍ0Á½ÖÖÖµÊµÏÖASKÐÅºÅµÄµ÷ÖÆ
**			  µ÷ÓÃWrite_ASF(u16 factor)À´¸Ä±ä·ùÖµ
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
****************************************************** ***************************************************/
void ASK_Init(void)
{
	AD9954_CS = 0;
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x02);//ÊÖ¶¯´ò¿ªOSK´ò¿ª
	AD9954_SENDBYTE(0x00);

	AD9954_SENDBYTE(0x20);//µ±ÊÜµ½UPDATEÐÅºÅ£¬×Ô¶¯Çå³þÏàÎ»ÀÛ¼ÓÆ÷		//
						  //	AD9954_SENDBYTE(0x00);//²»¸Ä±äÈÎºÎ

	AD9954_SENDBYTE(0x40);

	//¿ØÖÆÏàÎ»Æ«ÒÆ£¬ÒòÄ¬ÈÏÊä³öÊÇÓàÏÒ²¨£¬ËùÒÔÐè¿ØÖÆÏàÎ»ÀÛ¼ÓÆ÷µÄÀÛ¼ÓÏàÎ»Îª270
	AD9954_SENDBYTE(0X05);
	AD9954_SENDBYTE(0X30);
	AD9954_SENDBYTE(0X00);

	AD9954_CS = 1;
	UPDATE();
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid AM_Init(void)
** º¯Êý¹¦ÄÜ £ºAMµ÷ÖÆÐÅºÅÊä³ö³õÊ¼»¯
** º¯ÊýËµÃ÷ £ºAMµ÷ÖÆ£¬ÐèÒªÏÈÉèÖÃºÃÔØ²¨ÆµÂÊ£¬È»ºó¸Ä±äDAC±ÈÀýÒò×Ó scale factorÆäÎª14Î»,AMµÄ·ù¶ÈÖµ°´ÕýÏÒ¹æÂÉ±ä»»£¬¿ÉÒÔÊµÏÖAM
**            ×î¸ß¿ÉÒÔ²ÉÓÃ32µã²ÉÑù
**			  µ÷ÓÃWrite_ASF(u16 factor)À´¸Ä±ä·ùÖµ
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void AM_Init(void)
{
	AD9954_CS = 0;
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x02);//ÊÖ¶¯´ò¿ªOSK´ò¿ª
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);//²»¸Ä±äÈÎºÎ
	AD9954_SENDBYTE(0x40);

	AD9954_CS = 1;
	UPDATE();
}
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid Write_ASF(u16 factor)
** º¯Êý¹¦ÄÜ £º¸Ä±äscale factorÊýÖµ£¬¸Ä±äDACÊä³ö·ù¶È
** º¯ÊýËµÃ÷ £ºÐ´Èë×î´óÎª0X3FFF,×îÐ¡Îª0
** Èë¿Ú²ÎÊý £ºÎÞ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void Write_ASF(u16 factor)  //2ASK
{
	AD9954_CS = 0;
	/*	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x02);//ÊÖ¶¯´ò¿ªOSK´ò¿ª
	AD9954_SENDBYTE(0x00);

	AD9954_SENDBYTE(0x20);//µ±ÊÜµ½UPDATEÐÅºÅ£¬×Ô¶¯Çå³þÏàÎ»ÀÛ¼ÓÆ÷		//
	//	AD9954_SENDBYTE(0x00);//²»¸Ä±äÈÎºÎ

	AD9954_SENDBYTE(0x40);

	//¿ØÖÆÏàÎ»Æ«ÒÆ
	AD9954_SENDBYTE(0X05);
	AD9954_SENDBYTE(0X30);
	AD9954_SENDBYTE(0X00);	  */

	AD9954_SENDBYTE(0x02);//·ù¶È	
	AD9954_SENDBYTE(factor >> 8);
	AD9954_SENDBYTE(factor);
	AD9954_CS = 1;
	UPDATE();

}
/******************************************************************
*  Ãû    ³Æ£ºLinear_Sweep
*  ¹¦    ÄÜ£ºAD9954É¨ÆµÄ£Ê½Êä³ö
*  ²Î    Êý£ºMinFreq£ºÉ¨ÆµÏÂÏÞÆµÂÊ£¬MaxFreq£ºÉ¨ÆµÉÏÏÞÆµÂÊ
*            UpStepFreq£ºÏòÉÏÉ¨Æµ²½½ø£¬UpStepTime£ºÏòÉÏÉ¨ÆµµÄÌøÆµÊ±¼ä
*            DownStepFreq£ºÏòÏÂÉ¨Æµ²½½ø£¬DownStepTime£ºÏòÏÂÉ¨ÆµµÄÌøÆµÊ±¼ä
*            Mode£ºÉ¨ÆµÄ£Ê½
*  ·µ »Ø Öµ£ºÎÞ Linear_Sweep(100, 100000000, 100,  250,   100,250, 1);
*  Ëµ    Ã÷£ºÓëÆµÂÊÏà¹ØµÄ²ÎÊýµÄµ¥Î»¶¼ÊÇ MHz
*            Mode=DoubleScan(Ë«±ßÉ¨Æµ)Ê±£¬PS0=1£¬¼´ÎªÏòÉÏÉ¨Æµ,µ«ÅäÖÃÁË
*            ÏòÏÂÉ¨ÆµµÄ¼Ä´æÆ÷,¿ÉÒÔÖ±½ÓÍ¨¹ýPS0_WriteBit()À´¸Ä±äÉ¨Æµ·½Ïò
******************************************************************/
void Linear_Sweep( double  MinFreq, double MaxFreq, double  UpStepFreq, u8 UpStepTime,double  DownStepFreq, u8 DownStepTime,ScanMode      Mode )       //0ÏòÏÂÉ¨ÆµÄ£Ê½£¬1ÏòÉÏÉ¨ÆµÄ£Ê½£¬2Ë«±ßÉ¨Æµ

{
    uint32_t FTW_Vau;
    PS0=0;
    PS1=0;
    AD9954_CS=0;
    AD9954_SENDBYTE(CFR1);
    AD9954_SENDBYTE(0x02);
    AD9954_SENDBYTE(0x20);     //Linear_Sweep Enable
    AD9954_SENDBYTE(0);
    if (Mode & No_Dwell)
        AD9954_SENDBYTE(0x44);     //Comp Power-Down & Linear Sweep No Dwell
    else
        AD9954_SENDBYTE(0x40);     //Comp Power-Down
    
    //Ð´ÈëFTW0----×îÐ¡ÆµÂÊ
    FTW_Vau =  Get_FTW(MinFreq);;
    AD9954_SENDBYTE(FTW0);
    AD9954_SENDBYTE(FTW_Vau>>24);
    AD9954_SENDBYTE(FTW_Vau>>16);
    AD9954_SENDBYTE(FTW_Vau>>8);
    AD9954_SENDBYTE(FTW_Vau);
    
    //Ð´ÈëFTW1----×î´óÆµÂÊ
    FTW_Vau = Get_FTW(MaxFreq);
    AD9954_SENDBYTE(FTW1);
    AD9954_SENDBYTE(FTW_Vau>>24);
    AD9954_SENDBYTE(FTW_Vau>>16);
    AD9954_SENDBYTE(FTW_Vau>>8);
    AD9954_SENDBYTE(FTW_Vau);
    
    Mode &= 0x7F;
    
    if(Mode != UpScan)
    {
        //Ð´ÈëNLSCW----ÏÂ½µÆµÂÊ²½½øºÍÌøÆµÊ±¼ä
        FTW_Vau = Get_FTW(DownStepFreq);
        AD9954_SENDBYTE(NLSCW);
        AD9954_SENDBYTE(DownStepTime); //ÌøÆµÊ±¼ä£¨DownStepTime¸ö£¿£¿ÖÜÆÚ£©     
        AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
        AD9954_SENDBYTE(FTW_Vau>>16);
        AD9954_SENDBYTE(FTW_Vau>>8);
        AD9954_SENDBYTE(FTW_Vau);
        PS0=0;                         //ÏÂ½µÉ¨Æµ
    }
    if (Mode != DownScan)
    {
        //Ð´ÈëPLSCW----ÉÏÉýÆµÂÊ²½½øºÍÌøÆµÊ±¼ä
        FTW_Vau = Get_FTW(UpStepFreq);
        AD9954_SENDBYTE(PLSCW);
        AD9954_SENDBYTE(UpStepTime); //ÌøÆµÊ±¼ä£¨0XFF¸ö£¿£¿ÖÜÆÚ£©
        AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
        AD9954_SENDBYTE(FTW_Vau>>16);
        AD9954_SENDBYTE(FTW_Vau>>8);
        AD9954_SENDBYTE(FTW_Vau);
        PS0=1;                         //ÉÏÉýÉ¨Æµ
    }
    AD9954_CS=1;
		UPDATE();
}

/******************************************************************
*  Ãû    ³Æ£ºPS0_WriteBit
*  ¹¦    ÄÜ£ºÉèÖÃPS0µÄ×´Ì¬
*  ²Î    Êý£ºBitVal£ºPS0×´Ì¬
*  ·µ »Ø Öµ£ºÎÞ
*  Ëµ    Ã÷£ºPS0¿ÉÒÔ¿ØÖÆAD9954É¨Æµ·½Ïò
******************************************************************/
void PS0_WriteBit(BitAction BitVal)
{
    PS0=BitVal;
}

/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid DirectSwitchFSK(float f1,float f2,float f3,float f4)
** º¯Êý¹¦ÄÜ £ºËÄÏàFSKÐÅºÅÊä³ö
** º¯ÊýËµÃ÷ £ºÔÚËÄ¸öRAMÇø¸÷ÉèÖÃÁËÒ»¸öÆµÂÊÖµ£¬Í¨¹ý¸Ä±äPS0ºÍPS1µÄµçÆ½Ñ¡Ôñ¶ÔÓ¦µÄRAM¶ËÊä³öÏàÓ¦µÄÆµÂÊÖµÀ´ÊµÏÖFSK£¬Ò²¿ÉÒÔÊµÏÖ¶þÏîµÄFSK£»
**            Í¨¹ýÉèÖÃ¶¨Ê±Æ÷ÖÐ¶Ï¿ØÖÆPS0,PS1¹Ü½ÅµÄµçÆ½¾Í¿ÉÒÔ½«¶þ½øÖÆµÄ±àÂë×ª»¯ÎªFSKÐÅºÅÊä³ö
** Èë¿Ú²ÎÊý £ºfloat f1:ÆµÂÊ1
** 	      float f2:ÆµÂÊ2
** 	      float f3:ÆµÂÊ3
** 	      float f4:ÆµÂÊ4
** Òþº¬¿ØÖÆ   			PS0: 0   1	0	 1
¹Ü½Å²ÎÊý£º 				PS1: 0   0	1	 1
** ¶ÔÓ¦¿ØÖÆ RAM¶Î£º		 0   1	2	 3
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void DirectSwitchFSK(double f1,double f2,double f3,double f4)   
{
	u32 FTW_Vau;
	AD9954_CS = 0;

	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷   
	AD9954_SENDBYTE(0x00);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x40);   

	AD9954_SENDBYTE(0x01);//µØÖ·1Ð´²Ù×÷   
	AD9954_SENDBYTE(0x00);//   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0xA4);//8±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M   
	PS0=0;   
	PS1=0;   
	AD9954_SENDBYTE(0x07);   
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x00);//final address:0x000   
	AD9954_SENDBYTE(0x00);//start address:0x000;   
	AD9954_SENDBYTE(0x00);//RAM0¹¤×÷ÓÚÄ£Ê½0,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î   
	UPDATE();   
	FTW_Vau=Get_FTW(f1);
	AD9954_SENDBYTE(0x0b);
	AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
	AD9954_SENDBYTE(FTW_Vau>>16);
	AD9954_SENDBYTE(FTW_Vau>>8);
	AD9954_SENDBYTE(FTW_Vau);

	PS0=1;//ram1   
	PS1=0;   

	AD9954_SENDBYTE(0x08);   
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x01);//final address:0x0001   
	AD9954_SENDBYTE(0x04);//start address:0x0001   
	AD9954_SENDBYTE(0x00);//RAM1¹¤×÷ÓÚÄ£Ê½0,²»Í£ÁôÎ»Ã»ÓÐ¼¤»î   
	UPDATE();
	FTW_Vau=Get_FTW(f2);   
	AD9954_SENDBYTE(0x0b);   
	AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
	AD9954_SENDBYTE(FTW_Vau>>16);
	AD9954_SENDBYTE(FTW_Vau>>8);
	AD9954_SENDBYTE(FTW_Vau);  

	PS0=0;//ram2   
	PS1=1;   

	AD9954_SENDBYTE(0x09);   
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x02);//final address:0x0002   
	AD9954_SENDBYTE(0x08);//start address:0x0002   
	AD9954_SENDBYTE(0x00);   
	UPDATE();   
	FTW_Vau=Get_FTW(f3);   
	AD9954_SENDBYTE(0x0b);   
	AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
	AD9954_SENDBYTE(FTW_Vau>>16);
	AD9954_SENDBYTE(FTW_Vau>>8);
	AD9954_SENDBYTE(FTW_Vau); 
	PS0=1;//ram3   
	PS1=1;   
	AD9954_SENDBYTE(0x0a);   
	AD9954_SENDBYTE(0x01);//ramp rate=0x0010   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x03);//final address:0x0003   
	AD9954_SENDBYTE(0x0c);//start address:0x0003   
	AD9954_SENDBYTE(0x00);     
	UPDATE();   
	FTW_Vau=Get_FTW(f4);   
	AD9954_SENDBYTE(0x0b);   
	AD9954_SENDBYTE(FTW_Vau>>24);  //ÆµÂÊ²½½ø
	AD9954_SENDBYTE(FTW_Vau>>16);
	AD9954_SENDBYTE(FTW_Vau>>8);
	AD9954_SENDBYTE(FTW_Vau); 

	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷   
	AD9954_SENDBYTE(0x80);//´ò¿ªRAM¿ØÖÆÎ»Çý¶¯FTW   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x00);   
	AD9954_SENDBYTE(0x40);   
	AD9954_CS = 1;
	UPDATE();
}   
/*********************************************************************************************************
** º¯ÊýÃû³Æ £ºvoid LINEARSWEEP(float f1,float f2)
** º¯Êý¹¦ÄÜ £ºÏßÐÔÉ¨ÃæÊä³öÄ£Ê½
** º¯ÊýËµÃ÷ £ºÊ¹ÆµÂÊ°´Ô¤ÖÃµÄÄ£Ê½ÏßÐÔÉ¨ÃèÉÏÈ¥£

¬ÏêÏ¸²Î¼û¹Ù·½PDF
** Èë¿Ú²ÎÊý £ºfloat f1:ÆðÊ¼ÆµÂÊ
** 			  float f2:ÖÕÖ¹ÆµÂÊ
** ³ö¿Ú²ÎÊý £ºÎÞ
*********************************************************************************************************/
void LINEARSWEEP(float f1,float f2)//linear sweep mode
{
	u32 date;
	AD9954_SCLK=0;
	AD9954_RES=0;
	IOUPDATE=0;
	PS0=0;
	PS1=0;
	AD9954_CS=0;

	//linear sweep mode
	AD9954_SENDBYTE(0x00);//µØÖ·0Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x20);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x40);
	

	AD9954_SENDBYTE(0x01);//µØÖ·1Ð´²Ù×÷
	AD9954_SENDBYTE(0x00);//
	AD9954_SENDBYTE(0x00);
//	AD9954_SENDBYTE(0x24);//4±¶Æµ£¬´ò¿ªVCO¿ØÖÆ¸ßÎ»£¬ÏµÍ³Ê±ÖÓ±¶ÆµºóÎª400M
AD9954_SENDBYTE(PLL_MULTIPLIER<<3|0x04);
	date=10.7374*f1;
	AD9954_SENDBYTE(0x04);//FTW0µØÖ·
	AD9954_SENDBYTE((u8)(date>>24));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)(date>>16));
	AD9954_SENDBYTE((u8)(date>>8));
	AD9954_SENDBYTE((u8)date);
	date=10.7374*f2;
	AD9954_SENDBYTE(0x06);//FTW1µØÖ·
	AD9954_SENDBYTE((u8)(date>>24));//ÆµÂÊ¿ØÖÆ×Ö
	AD9954_SENDBYTE((u8)(date>>16));
	AD9954_SENDBYTE((u8)(date>>8));
	AD9954_SENDBYTE((u8)date);
	AD9954_SENDBYTE(0x07);//NLSCW
	AD9954_SENDBYTE(0x01);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x1b);
	AD9954_SENDBYTE(0xf4);
	AD9954_SENDBYTE(0x08);//RLSCW
	AD9954_SENDBYTE(0x01);
	AD9954_SENDBYTE(0x00);
	AD9954_SENDBYTE(0x01);
	AD9954_SENDBYTE(0xa3);
	AD9954_SENDBYTE(0x6f);
	
	AD9954_CS=1;
	UPDATE();
}
