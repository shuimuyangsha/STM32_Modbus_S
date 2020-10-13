#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 

//////////////////////////////////////////////////////////////////////////////////	 

//AGVС������2��ʼ������ģ��		   

////////////////////////////////////////////////////////////////////////////////// 	  
 




 

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
//��ʼ��IO ����2 
//bound:������
void uart_init(u32 bound)
{
//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	//ʹ��USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
 	USART_DeInit(USART2);                                   //��λ����2
	 
//USART2_TX   PA.2 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;              //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //��ʼ��PA2
   
//USART2_RX	  PA.3    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //��ʼ��PA3

//Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			    //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;                                     //һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                             //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ

   	USART_Init(USART2, &USART_InitStructure);                                       //��ʼ������2
   	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                                  //�����ж�2
		USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//�رմ���2�����ж�
   	USART_Cmd(USART2, ENABLE);                                                      //ʹ�ܴ���2
}

void USART2_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//�����ж�
	{

		void ModRcv(void);

	} 
	if(USART_GetITStatus(USART2,USART_IT_TC) != RESET)
	{
		void ModSend(void);//����modbus��Ϣ����
	}	
} 
 

