#ifndef __AD9954_H
#define __AD9954_H	 
#include "sys.h"
/*
** 函数名称 ：void GPIO_AD9954_Init(void)
** 函数功能 ：AD9954接口IO初始化
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



//#define AD9954_CS 	PAout(3)// PE5	
//#define AD9954_SCLK PAout(4)// PE5	
//#define AD9954_SDIO PAout(5)// PE5	
//#define AD9954_OSK 	PCout(0)// PE5	
//#define PS0 				PAout(2)// PB5
//#define PS1 				PBout(10)// PE5	
//#define IOUPDATE 		PAout(7)// PE5	

//#define AD9954_SDO	PAin(8)
//#define AD9954_IOSY PBout(1)// PE5	
//#define AD9954_RES 	PAout(6)// PE5	
//#define AD9954_PWR 	PBout(0)// PE5

#define AD9954_CS 	PFout(0)// PE5	
#define AD9954_SCLK PFout(1)// PE5	n
#define AD9954_SDIO PFout(2)// PE5	
#define AD9954_OSK 	PFout(3)// PE5	
#define PS0   			PFout(4)// PB5
#define PS1 				PFout(5)// PE5	
#define IOUPDATE 		PFout(12)// PE5	

#define AD9954_SDO	PFin(13)
#define AD9954_IOSY PFout(14)// PE5	
#define AD9954_RES 	PFout(15)// PE5	
#define AD9954_PWR 	PGout(0)// PE5

#define CFR1	0X00			
#define CFR2    0X01
#define ASF     0X02
#define ARR     0X03
#define FTW0    0X04
#define POW0    0X05
#define FTW1    0X06
#define NLSCW   0X07
#define PLSCW   0X08
#define RSCW0   0X07
#define RSCW1   0X08
#define RSCW2   0X09
#define RSCW3   0X0A
#define RAM     0X0B

#define No_Dwell 0X80

typedef enum {
DownScan  =   0,
UpScan,
DoubleScan
}ScanMode;


void AD9954_Init(void);
void AD9954_SETFRE(double f);//single tone
void Single_Tone(double Freq);
//void AD9954_Init(void);
//void Single_Tone(double Freq);
void Linear_Sweep(double  MinFreq,double MaxFreq,double UpStepFreq, u8 UpStepTime,double  DownStepFreq, u8 DownStepTime,ScanMode  Mode);//扫频
void PS0_WriteBit(BitAction BitVal);
void SetASF(u16 data);
void ASK_Init(void);
void Generate_PSK(float f, u16 *phase);
void DirectSwitchFSK(double f1, double f2, double f3, double f4);
void DirectSwitchPSK(float f, float phase1, float phase2, float phase3, float phase4);
void AM_Init(void);
void Write_ASF(u16 factor);  //2ASK
u32 Read_Vau1(void);
u32 Read_Vau2(void);
		void LINEARSWEEP(float f1,float f2);//linear sweep mode
#endif

