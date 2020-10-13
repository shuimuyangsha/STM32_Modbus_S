/***********************************************
modbus 主从机程序，对下使用从机RXD2，对上使用RXD1

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
	delay_init();	     //延时初始化
	NVIC_Configuration();
	Timer2_Init(175,720);
	uart_init(19200);	 	//串口初始化为19200
	
	ModInit(0x01);  //站号  01


	LCD_Init();
	LCD_Clear(WHITE);																				    

	while(1)
	{
		POINT_COLOR = RED;
		for(i=0;i<6;i++)
		{
			//rs485buf[i]=cnt+i;//填充发送缓冲区
			LCD_ShowxNum(60+i*32,190,HoldReg[i],3,16,0X80);	//显示数据
		}
		POINT_COLOR=RED;			
		for(j=0;j<5;j++)
		{
			//rs485buf[i]=cnt+i;//填充发送缓冲区
			LCD_ShowxNum(60+j*32,206,HoldReg[j+5],3,16,0X80);	//显示数据
		} 
		for(k=0;k<5;k++)
		{
			//rs485buf[i]=cnt+i;//填充发送缓冲区
			LCD_ShowxNum(60+k*32,222,HoldReg[k+10],3,16,0X80);	//显示数据
		} 			
	}
}












