#include "timer.h"
#include "delay.h"
#include "modbus_rtu.h"

// extern u16 flag_wcz;
// extern u8 index_sx_zhu;
// extern u8 index_sx_zi;
// extern u8 index_qd;
// extern u8 index_qd_nz;
// extern u8 index_sx_zi_nz;
// extern u8 LCD_BG;	 //��ʱ�ر���
// extern u8 FLAG_comerr;

//ͨ�ö�ʱ���жϳ�ʼ��	ͨ�ö�ʱ�� 2��3��4
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��2,3!






void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_DeInit(TIM2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��	 
	TIM_TimeBaseStructure.TIM_Period = arr;              //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc;            //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;         //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);      //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_ARRPreloadConfig(TIM2,ENABLE);//Ԥװ��ʹ��
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//�������жϱ�־λ
	
	//TIM_SetCounter(TIM2,0);//��ʼ��TIM2�������ĳ�ʼֵΪ0

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//��ʱ������жϹر�

  TIM_Cmd(TIM2, DISABLE);  //��ʱ��2����						 
}











