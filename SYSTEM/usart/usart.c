#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 

//////////////////////////////////////////////////////////////////////////////////	 

//AGV小车串口2初始化程序模块		   

////////////////////////////////////////////////////////////////////////////////// 	  
 




 

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//初始化IO 串口2 
//bound:波特率
void uart_init(u32 bound)
{
//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	//使能USART2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
 	USART_DeInit(USART2);                                   //复位串口2
	 
//USART2_TX   PA.2 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;              //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化PA2
   
//USART2_RX	  PA.3    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化PA3

//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			    //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化VIC寄存器
  
//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;                                     //一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式

   	USART_Init(USART2, &USART_InitStructure);                                       //初始化串口2
   	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                                  //开启中断2
		USART_ITConfig(USART2,USART_IT_TXE,DISABLE);//关闭串口2发送中断
   	USART_Cmd(USART2, ENABLE);                                                      //使能串口2
}

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收中断
	{

		void ModRcv(void);

	} 
	if(USART_GetITStatus(USART2,USART_IT_TC) != RESET)
	{
		void ModSend(void);//用于modbus信息接收
	}	
} 
 

