#include "timer.h"
#include "delay.h"
#include "modbus_rtu.h"

// extern u16 flag_wcz;
// extern u8 index_sx_zhu;
// extern u8 index_sx_zi;
// extern u8 index_qd;
// extern u8 index_qd_nz;
// extern u8 index_sx_zi_nz;
// extern u8 LCD_BG;	 //延时关背光
// extern u8 FLAG_comerr;

//通用定时器中断初始化	通用定时器 2、3、4
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器2,3!






void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_DeInit(TIM2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能	 
	TIM_TimeBaseStructure.TIM_Period = arr;              //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc;            //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;         //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);      //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ARRPreloadConfig(TIM2,ENABLE);//预装载使能
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除溢出中断标志位
	
	//TIM_SetCounter(TIM2,0);//初始化TIM2计数器的初始值为0

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//定时器溢出中断关闭

  TIM_Cmd(TIM2, DISABLE);  //定时器2禁能						 
}











