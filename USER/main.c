#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "uart4.h"
#include "timer.h"
#include "ads115.h"
#include "oled.h"
#include "dma.h"
#include "ADS1292.h"
#include "exti.h"
#include "mlx90614.h" 
#include "beep.h"
#include "adc.h"
#include "stdio.h"
#include "LCD.h"
#include "AD9954.h"
/************************************************

2020������ƾ���-TI��  �ۺϲ��Գ���
˵����������ֻ��2020 TI����Ҫ�õ��Ĵ��������ܲ��Գ���
���ߣ�����Ȼ
�汾��V1.1

					Ӳ��˵��
---------------------------------------------------
Ӳ������		|	���		|	����	|	��ע
---------------------------------------------------
USART		|	RXD			|	PA10	|
			|	TXD			|	PA9		|
---------------------------------------------------
UART4		|	RXD			|	PC11	|
			|	TXD			|	PC10	|
---------------------------------------------------
KEY			|	KEY_UP		|	PA4		|
			|	KEY_LEFT	|	PA5		|
			|	KEY_RIGHT	|	PA6		|
			|	KEY_DOWN	|	PA7		|
			|	KEY_SURE	|	PC4		|
			|	KEY_CANCEL	|	PC5		|
---------------------------------------------------
LED			|	LED_R		|	PC0		|
			|	LED_G		|	PC1		|
			|	LED_B		|	PC2		|
---------------------------------------------------
BEEP		|	BEEP		|	PC3		|
---------------------------------------------------
PWM			|	TIME4_CH1	|	PB6		|
			|	TIME4_CH2	|	PB7		|
			|	TIME4_CH3	|	PB8		|
			|	TIME4_CH4	|	PB9		|
---------------------------------------------------
ADC			|	ADC1_CH0	|	PA0		|
			|	ADC1_CH1	|	PA1		|
			|	ADC1_CH2	|	PA2		|
			|	ADC1_CH3	|	PA3		|
---------------------------------------------------
ADS1292		|	PWDN		|	PB10	|
			|	START		|	PB11	|
			|	DRDY		|	PC6		|
			|	CS0			|	PB12	|
			|	MOSI		|	PB15	|
			|	SCK			|	PB13	|
			|	MISO		|	PB14	|
---------------------------------------------------
MLX90614	|	SCL			|	PA8		|
			|	SDA			|	PC9		|
---------------------------------------------------
ADS1115 	|	SCL			|	PD2		|
			|	SDA			|	PC12	|
---------------------------------------------------
1.3 OLED	|	SCL			|	PB0		|
			|	SDA			|	PB1		|
---------------------------------------------------
FLASH		|	SCL			|	PC8		|
			|	SDA			|	PC7		|
---------------------------------------------------
USB    		|	USB_DP		|	PA12	|
			|	USB_DM		|	PA11	|


����������ļ������ݵ����壺
Code����ʾ������ռ�� FLASH �Ĵ�С��FLASH��
RO-data���� Read Only-data�� ��ʾ������ĳ������� const ���ͣ�FLASH��
RW-data���� Read Write-data�� ��ʾ�ѱ���ʼ����ȫ�ֱ�����SRAM��
ZI-data���� Zero Init-data�� ��ʾδ����ʼ����ȫ�ֱ���(SRAM)

************************************************/
extern unsigned char Plane_BMP[];   

int main(void)
{		
	u16 result;         	//����ADS1115�ı���
	float result_val;

	u16 adcx;           	//����MCU_ADC�ı���

	u16 len,t;          	//���ڴ���4�ı���

	u8 key;             	//���ڰ����ı���

	u8 test_flag = 0;   	//������ѡ�񰴼�

	float temp;         	//LMT70�¶ȱ���

	u8 dtbuf[50];       	//OELD�ַ�����ʾ���� 

	u16 pwm = 1500;     	//������Ʊ���   0-180��  500-2500

	u8 i,sum;	    	//����ADS1292�ı���
	u8 data_to_send[60];	//���ڷ��ͻ���
	u32 cannle[2];	    	//�洢����ͨ��������
	s32	p_Temp[2];	    	//���ݻ���	
  char aa[10];

  float temp1;

	data_to_send[0]=0xAA;  	//��λ��������������ͷ
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF1;	
	data_to_send[3]=8;	 
	 
	delay_init();	    	 		//��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	     		//����һ��ʼ��Ϊ115200
	uart4_init(115200);	     		//�����ĳ�ʼ��Ϊ115200
	LED_Init();			     		//LED�˿ڳ�ʼ��
	BEEP_Init(); 					//��������ʼ��
	KEY_Init();  					//������ʼ��
  LCD_Init();

	TIM3_Int_Init(4999,7199);				//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms  
	TIM4_PWM_Init(19999,71); 				//����Ƶ��PWMƵ��=72000000/72=1000000/20000=50Hz
	ADS1115_Init();    						//ADS1115��ʼ��

	DMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)data_to_send);	//����1DMA����
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 					//DMAʹ��	
	ADS1292_Init();	        				//��ʼ��ads1292
	EXTIX_Init();         					//��ʼ���ⲿ�ж����� 
  AD9954_Init();
	Adc_Init();		  						//ADC��ʼ��
	SMBus_Init();           				//MLX90614��ʼ��
	
//	  POINT_COLOR=RED;//��������Ϊ��ɫ
//    LCD_ShowNum(50,50,666,16,16);
//	  delay_ms(1000);
		while(Set_ADS1292_Collect(0))			//0 �����ɼ�  1 1mV1Hz�ڲ������ź�  2 �ڲ��̽���������
	{	
		printf("1292�Ĵ�������ʧ��\r\n");
		delay_ms(1000);		
	}	
	printf("�Ĵ������óɹ�\r\n");
	delay_ms(1000);	       	//��ʱ1��
		
		
while(1){

///*----------------------ADS1292����-------------------*/		  

			if(ads1292_recive_flag)
			{										
				cannle[0]=ads1292_Cache[3]<<16 | ads1292_Cache[4]<<8 | ads1292_Cache[5];	//��ȡԭʼ����		
				cannle[1]=ads1292_Cache[6]<<16 | ads1292_Cache[7]<<8 | ads1292_Cache[8];
//				POINT_COLOR=RED;//��������Ϊ��ɫ
//        LCD_ShowNum(50,50,cannle[0],16,16);
				p_Temp[0] = get_volt(cannle[0]);		//�Ѳɵ���3���ֽ�ת���з���32λ��
				p_Temp[1] = get_volt(cannle[1]);		//�Ѳɵ���3���ֽ�ת���з���32λ��

				//�з�����Ϊ��תΪ�޷��ţ��޷�����Ϊ�߼�����
				cannle[0] = p_Temp[0];
				cannle[1]	= p_Temp[1];
				data_to_send[4]=cannle[0]>>24;			//25-32λ
				data_to_send[5]=cannle[0]>>16;  		//17-24
				data_to_send[6]=cannle[0]>>8;			//9-16
				data_to_send[7]=cannle[0]; 				//1-8

				data_to_send[8]=cannle[1]>>24;			//25-32λ
				data_to_send[9]=cannle[1]>>16;  		//17-24
				data_to_send[10]=cannle[1]>>8;			//9-16
				data_to_send[11]=cannle[1];			 	//1-8

				for(i=0;i<12;i++)
				{sum += data_to_send[i];							
				data_to_send[12] = sum;					//У���																		
				DMA_Enable(DMA1_Channel4,13);			//����1DMA 
				}									
				ads1292_recive_flag=0; 
				sum = 0;	
			}

		}			  
		
	}	 



