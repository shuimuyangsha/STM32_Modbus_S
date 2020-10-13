/***********************************************
modbus ���ӻ����򣬶���ʹ�ôӻ�RXD2������ʹ��RXD1

************************************************/

#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "modbus_rtu.h"
#include "timer.h"
#include "lcd.h"



u16 HoldReg[100];




int main(void)
{
  
	u8 i=0,j=0,k=0;
	//SystemInit();
	delay_init();	     //��ʱ��ʼ��
	NVIC_Configuration();
	Timer2_Init(175,720);
	uart_init(19200);	 	//���ڳ�ʼ��Ϊ19200
	
	ModInit(0x01);  //վ��  01


	LCD_Init();
	LCD_Clear(WHITE);																				    

	while(1)
	{
		POINT_COLOR = RED;
		for(i=0;i<6;i++)
		{
			//rs485buf[i]=cnt+i;//��䷢�ͻ�����
			LCD_ShowxNum(60+i*32,190,HoldReg[i],3,16,0X80);	//��ʾ����
		}
		POINT_COLOR=RED;			
		for(j=0;j<5;j++)
		{
			//rs485buf[i]=cnt+i;//��䷢�ͻ�����
			LCD_ShowxNum(60+j*32,206,HoldReg[j+5],3,16,0X80);	//��ʾ����
		} 
		for(k=0;k<5;k++)
		{
			//rs485buf[i]=cnt+i;//��䷢�ͻ�����
			LCD_ShowxNum(60+k*32,222,HoldReg[k+10],3,16,0X80);	//��ʾ����
		} 			
	}
}












