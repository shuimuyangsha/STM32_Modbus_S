#include "modbus_rtu.h"
#include "usart.h"
#include "timer.h"
/*  此Modbus协议暂时只支持RTU模式，只支持作为Modbus从设备。
	暂时支持的功能码（16进制）如下表所示：
	01.读线圈状态（读多个输出位的状态，有效地位为0-31）
	02.读输入位状态（读多个输入位的状态，有效地位为0-31）
	03.读保持寄存器（读多个保持寄存器的数值，有效地位为0-99）
	04.读输入寄存器（读多个输入寄存器的数值，有效地址为0-1）
	05.强制单个线圈（强制单个输出位的状态，有效地位为0-31）
	06.预制单个寄存器（设定一个寄存器的数值，有效地址为0-99）
	0F.强制多个线圈（强制多个输出位的状态，有效地址为0-31）
	10.预制多个寄存器（设定多个寄存器的数值，有效地址为0-99）

	暂时支持的错误代码为：
	01 不合法功能代码从机接收的是一种不能执行功能代码。发出查询命令后，该代码指示无程序功能。（不支持的功能代码）
    02 不合法数据地址接收的数据地址，是从机不允许的地址。（起始地址不在有效范围内）
    03 不合法数据查询数据区的值是从机不允许的值。（在起始地址的基础上，这个数量是不合法的）

	供用户调用的函数有：
	1.void ModInit(u8 Id);//用于Modbus初始化，在函数调用前，必须初始化函数，用于Main函数中
	2.void ModRcv(void);  //用于modbus信息接收，放在串口接收中断
	3.void ModSend(void); //用于modbus信息接收,放在串口发送中断
	 例如：	void USART1_IRQHandler(void)   //USART1中断
	{

	  if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
		{
			void ModRcv(void);
			……
			……
			……
		}

	  if(USART_GetITStatus(USART1,USART_IT_TC)!=RESET)
		{
			void ModSend(void);//用于modbus信息接收
			……
			……
			……
		}
	  
	}																				 */



//modbus用通讯参数
u8 Tim_Out;         //大于3.5个字符时间，保守取3ms (波特率9600的时候大约2点几毫秒)
u8 Rcv_Complete;    //一帧是否已经接受完成
u8 Send_Complete;   //一帧是否已经发送完成
u8 Com_busy;        //通讯繁忙，表示上一帧还未处理结束
u8 Rcv_Buffer[210]; //用来存放接收到的完整的一帧数据	（第一个字节用来存放接收到的有效字节数，也就是数组中的有效字节数）					  
u8 Send_Buffer[210];//用来存放待发送的完整的一帧数据（第一个字节用来存放待发送的有效字节数，也就是数组中的有效字节数）
u8 Rcv_Data;        //用来存放接收的一个字节
u8 Send_Data;       //用来存放要发送的一字节
u8 Mod_Id;          //用来标志作为从站的站号
u8 Rcv_Num;         //用来表示接收的一帧的有效字节数（从功能码到CRC校验）
u8 Send_Num;        //用来表示待发送的一帧的字节数																				
u8 *PointToRcvBuf;  //用来指向接收的数据缓存
u8 *PointToSendBuf; //用来指向带发送的数据缓存
u8 Comu_Busy;       //用来表示能否接收下一帧数据
u8 HaveMes;         //信息接收指示（=1表示接收到信息）
extern u16 HoldReg[100];


//CRC校验查表用参数
/* CRC 高位字节值表*/
    static u8 auchCRCHi[] = {
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
    0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
    0x80,0x41,0x00,0xC1,0x81,0x40
    } ;
/* CRC低位字节值表*/
    static u8 auchCRCLo[] = {
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,
    0x07,0xC7,0x05,0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,
    0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
    0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,
    0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,
    0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
    0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,
    0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
    0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,
    0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,
    0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
    0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
    0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,
    0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
    0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,
    0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
    0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,
    0x70,0xB0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
    0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
    0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,
    0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,
    0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
    0x43,0x83,0x41,0x81,0x80,0x40
    } ;

//声明modbus的函数
void ModInit(u8 Id);                               //用于Modbus初始化，参数Id为站号（1-255）
void ModRcv(void);                                 //用于modbus信息接收
void ModSend(void);                                //用于modbus信息接收
void MessageHandle(u8 *pointer_in,u8 *pointer_out);//处理收到的信息帧
void ReadOutputBit(u8 *pointer_1,u8 *pointer_2);   //读线圈
void ReadInputBit(u8 *pointer_1,u8 *pointer_2);    //读输入位
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2);  //读保持寄存器	
void ReadInputReg(u8 *pointer_1,u8 *pointer_2);    //读输入寄存器
void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2); //强制单个线圈
void PresetSingleReg(u8 *pointer_1,u8 *pointer_2); //预制单个寄存器
void ForceMulCoil(u8 *pointer_1,u8 *pointer_2);    //强制多个线圈
void PresetMulReg(u8 *pointer_1,u8 *pointer_2);    //预制多个寄存器
void ErrorHandle(u8 Mode,u8 *Pointer);             //错误信息帧处理
u16 CRC16(u8 *puchMsgg,u8 usDataLen);              //用于计算CRC校验码

 /* 函数功能：用于Modbus初始化
 	函数输入：Id为Modbus站号。												
	函数输出：无。                                                                                      */
  void ModInit(u8 Id)
  {
  	//modbus参数初始化
	  PointToRcvBuf=Rcv_Buffer;
    PointToSendBuf=Send_Buffer;
    Send_Num=1;//发送的数据顺序（输出数组的第几个数）
	  Mod_Id=Id;//站号设置
    Rcv_Buffer[1]=Mod_Id;
    Send_Buffer[1]=Mod_Id;
	  Comu_Busy=0;
  }
  
 /* 函数功能：用于Modbus信息接收
 	函数输入：无。														    
	函数输出：无。                                                                                      */
  void ModRcv(void)
  {
		HaveMes=1;//表示接收到了信息
		Rcv_Data=USART_ReceiveData(USART2);
		if(Comu_Busy!=1)//如果不忙，可以接收下一帧信息
		{
			TIM_Cmd(TIM2, DISABLE);//关闭时钟
	    TIM_SetCounter(TIM2,0);//初始化TIM2计数器的初始值为0
			if((Tim_Out!=0)&&(Rcv_Data==Mod_Id))//如果间隔时间超过了3.5个字符，同时接受的字节和自己的站号一致，则认为接收开始
			{
				Rcv_Complete=0; //表示数据帧接收开始
				Rcv_Num=0;      //接收数据个数初始化
				Rcv_Num++;      //同时个数加一
			}
			if((0==Tim_Out)&&(0==Rcv_Complete))//如果处于接收一帧的正常过程中
			{
				if(Rcv_Num<100)
				{
					Rcv_Buffer[Rcv_Num+1]=Rcv_Data;//将数据放入接收数组中
					Rcv_Num++;//同时个数加一	
				}
				else
				{
					Rcv_Complete=1;
					Comu_Busy=1;
					Rcv_Buffer[0]=Rcv_Num;
					*(PointToSendBuf+2)=*(PointToRcvBuf+2);//获取功能码
					ErrorHandle(6,PointToSendBuf);//表示超出了字节数(从机设备忙碌)
					Rcv_Num=0;
				}
			}
			Tim_Out=0;
		  TIM_Cmd(TIM2, ENABLE);//开启4.5ms计时（三个半字符的保守估计）
		}
  }

 /* 函数功能：用于Modbus信息发送
 	函数输入：无。														    
	函数输出：无。																	   */
  void ModSend(void)	
 {	
 	Send_Data=*(PointToSendBuf+Send_Num);
	USART_ClearFlag(USART2,USART_FLAG_TC);                 //读取USART_SR
  USART_SendData(USART2,Send_Data);
	Send_Num++;
	if(Send_Num>(*PointToSendBuf))//发送已经完成
	{
		Comu_Busy=0;
		*PointToSendBuf=0;
		Rcv_Num=0;
		Send_Num=1;
		//启动数据发送
	  USART_ITConfig(USART2, USART_IT_TC, DISABLE);//关闭数据发送中断	
	}
 }
                                                                                     
 /* 函数功能：Modbus专用定时器（TIM2）
 	函数输入：无。														    
	函数输出：无。													  */
void TIM2_IRQHandler(void)															  
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源
	  Tim_Out=1;
		TIM_Cmd(TIM2,DISABLE);//关闭时钟
		TIM_SetCounter(TIM2,0);//初始化TIM2计数器的初始值为0
		Rcv_Complete=1;
		Rcv_Buffer[0]=Rcv_Num;

		if(HaveMes!=0&&Rcv_Num>4)
		{
			Comu_Busy=1;
			MessageHandle(PointToRcvBuf,PointToSendBuf);
		}
		
	}	
}

	
	
	
 /* 函数功能：CRC校验用函数
 	函数输入：puchMsgg是要进行CRC校验的消息，usDataLen是消息中字节数														    
	函数输出：计算出来的CRC校验码。                                                                                      */
u16 CRC16(u8 *puchMsgg,u8 usDataLen)//puchMsgg是要进行CRC校验的消息，usDataLen是消息中字节数 
{
    u8 uchCRCHi = 0xFF ; /* 高CRC字节初始化*/
    u8 uchCRCLo = 0xFF ; /* 低CRC 字节初始化*/
    u8 uIndex ; /* CRC循环中的索引*/
    while (usDataLen--) /* 传输消息缓冲区*/
    {
				uIndex = uchCRCHi ^ *puchMsgg++ ; /* 计算CRC */
				uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
				uchCRCLo = auchCRCLo[uIndex] ;
    }
    return ((uchCRCHi << 8) | uchCRCLo) ;
}

 /* 函数功能：对输入的信息帧进行处理，按照功能码不同，调用不同的函数处理
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */
void MessageHandle(u8 *pointer_in,u8 *pointer_out)		  
 {
 	u16 CalKey;//计算出来的校验值
	u16 RcvKey;//接收到的校验值
	
	HaveMes=0;//清除信息位
	//获取接收到的校验值
	RcvKey=(u16)*(pointer_in+(*pointer_in-1));
	RcvKey=RcvKey<<8;
	RcvKey=RcvKey|(u16)*(pointer_in+(*pointer_in));

	CalKey=CRC16(pointer_in+1,*pointer_in-2);
	if(CalKey==RcvKey)
	{
		switch(*(pointer_in+2))//第二个字节为功能码
		{
			case 0x01:ReadOutputBit(pointer_in,pointer_out);  //读输出线圈
				break;
			case 0x02:ReadInputBit(pointer_in,pointer_out);   //读输入位
				break;
			case 0x03:ReadHoldingReg(pointer_in,pointer_out); //读保持寄存器
				break;
			case 0x04:ReadInputReg(pointer_in,pointer_out);   //读输入寄存器
				break;
			case 0x05:ForceSingleCoil(pointer_in,pointer_out);//强制单个线圈状态
				break;
			case 0x06:PresetSingleReg(pointer_in,pointer_out);//预制单个寄存器
				break;
			case 0x0F:ForceMulCoil(pointer_in,pointer_out);   //强制多个线圈
				break;
			case 0x10:PresetMulReg(pointer_in,pointer_out);   //预制多个寄存器
				break;												 
			default: 
			{
				*(pointer_out+2)=*(pointer_in+2);//获取功能码
				ErrorHandle(1,pointer_out);      //功能码错误 
			}	
				break;
		}
	}
	else
	{
		Comu_Busy=0;
	} 		
 }


  /* 函数功能：读取线圈状态
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void ReadOutputBit(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0; //待读取线圈起始地址（GPIO_X,X为A，B两个端口，每个端口16位，对应地址0——31）												
	u16 Num=0;     //要读取的线圈个数
	u8 Byte=0;     //要读取的线圈个数总共占用的字节数；
	u32 PortTemp;  //用来存放从端口取过来的数据
	u16 ReadData=0;//用来临时存放从端口读来的数据
	u16 SendKey;   //要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到线圈地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到要读取的线圈个数
	*(pointer_2+2)=0x01;                               //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{
		if(Address<32) //只要地址小于32，就是合法地址
		{
			if(Address+Num<=32&&Num>0) //只要地址加数量大于0小于80，就是合法数量
			{	
				//用于for循环
				u8 i;
				u8 j;

				Byte=Num/8;
				if(Num%8!=0)
				Byte++;//如果不整除的话，加一个字节，剩余的高位补零
				*(pointer_2+3)=Byte;//第四个字节为要发送的个数
				*(pointer_2)=1+1+1+Byte+2;//有效字节个数等于丛机地址+功能码+字节个数+线圈信息+CRC校验
	
				//将端口C和D的数据预先读入到临时的数据缓存中
				ReadData=GPIO_ReadOutputData(GPIOD);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadOutputData(GPIOC);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//将PortTemp中的数据处理成需要的位数，再装入输出数据缓存中
				PortTemp=PortTemp<<(31-(Address+Num-1));
				PortTemp=PortTemp>>((31-(Address+Num-1))+Address);
	
				//将数据一个字节一个字节装到发送数据缓存数组中
				
				for(i=4,j=Byte;j>0;j--,i++)
				{
					*(pointer_2+i)=(u8)(PortTemp&0x00FF);//截取一个字节的长度，存放起来
					PortTemp=PortTemp>>8;//再将数据向右移动8位
				}
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
									
			}
			else
			{
				 ErrorHandle(3,pointer_2);//错误读取数量
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


 /* 函数功能：读取输入状态
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void ReadInputBit(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;//待读取输入位起始地址（GPIO_X,X为C，D两个端口，每个端口16位，对应地址0-31）
	u16 Num=0;//要读取的输入位个数
	u8 Byte=0;//要读取的输入位个数总共占用的字节数；
	u32 PortTemp;//用来存放从端口取过来的数据，临时计算用
	u16 ReadData=0;//用来临时存放从端口读来的数据
	u16 SendKey;//要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到输入位地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到要读取的输入位个数
	*(pointer_2+2)=0x02;                               //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{

		if(Address<32) //只要地址小于32，就是合法地址
		{
			if(Address+Num<=32&&Num>0) //只要地址加数量大于0小于80，就是合法数量
			{
				//用于for循环
				u8 i;
				u8 j;
				
				Byte=Num/8;
				if(Num%8!=0)
				Byte++;//如果不整除的话，加一个字节，剩余的高位补零
				*(pointer_2+3)=Byte;//第四个字节为要发送的个数
				*(pointer_2)=1+1+1+Byte+2;//有效字节个数等于丛机地址+功能码+字节个数+输入位信息+CRC校验
	
				//将端口A和B的数据预先读入到临时的数据缓存中
				ReadData=GPIO_ReadInputData(GPIOB);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadInputData(GPIOA);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//将PortTemp中的数据处理成需要的位数，再装入输出数据缓存中
				PortTemp=PortTemp<<(31-(Address+Num-1));
				PortTemp=PortTemp>>((31-(Address+Num-1))+Address);
	
				//将数据一个字节一个字节装到发送数据缓存数组中
				for(i=4,j=Byte;j>0;j--,i++)
				{
					*(pointer_2+i)=(u8)(PortTemp&0x00FF);//截取一个字节的长度，存放起来
					PortTemp=PortTemp>>8;//再将数据向右移动8位
				}
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
				 
			}
			else
			{
				 ErrorHandle(3,pointer_2);//错误读取数量
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


/* 函数功能：读取保持寄存器
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。
	                                                                                      */
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;//待读取寄存器起始地址（HoldReg[i],i为0-99对应地址从0到99）
	u16 Num=0;    //要读取的寄存器个数
	u16 SendKey;  //要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到寄存器起始地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到要读取的寄存器个数
	*(pointer_2+2)=0x03;                               //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{

		if(Address<100) //只要地址小于100，就是合法地址
		{
			if(Address+Num<=100&&Num>0) //只要地址加数量大于0小于100，就是合法数量
			{
				
 				//用于for循环
				u8 i;
				u8 j;

				*(pointer_2+3)=Num*2;      //第四个字节为要发送的字节个数
				*(pointer_2)=1+1+1+Num*2+2;//有效字节个数等于丛机地址+功能码+字节个数+寄存器信息+CRC校验
	
	
				for(i=Address,j=4;i<Address+Num;i++,j+=2)
				{
					*(pointer_2+j)=(u8)(HoldReg[i]>>8);//先放高位
					*(pointer_2+j+1)=(u8)(HoldReg[i]&0x00FF);//再放低位
				}
					
				//写入校验码						   
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
						
			}
			else
			{
				 ErrorHandle(3,pointer_2);//错误读取数量
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


/* 函数功能：读取输入寄存器（模拟量输入）
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。
	                                                                                      */
 void ReadInputReg(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;//待读取寄存器起始地址（HoldReg[i],i为0-99对应地址从0到99）
	u16 Num=0;//要读取的寄存器个数
	u16 SendKey;//要发送数据的校验值
	u32 PortTemp;//用来存放从端口取过来的数据，临时计算用
	u16 ReadData=0;//用来临时存放从端口读来的数据
	u32 CalTemp=0;//用来临时计算

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到寄存器起始地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到要读取的寄存器个数
	*(pointer_2+2)=0x04;                               //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{
		if(Address<2) //只要地址小于2，就是合法地址
		{
			if(Address+Num<=2&&Num>0) //只要地址加数量大于0小于2，就是合法数量
			{
				//用于for循环
				u8 i;
				u8 j;
								
				*(pointer_2+3)=Num*2;//第四个字节为要发送的字节个数
				*(pointer_2)=1+1+1+Num*2+2;//有效字节个数等于丛机地址+功能码+字节个数+寄存器信息+CRC校验
	
				//将端口ADC1和ADC2的数据预先读入到临时的数据缓存中
				ReadData=0x01;    //2014-9-28
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=0x02;    //2014-9-28
				PortTemp=PortTemp|(u32)(ReadData);
				
				//将PortTemp中的数据先进行预处理
				PortTemp=PortTemp<<(Address*16);
				for(i=4,j=Num*2;j>0;i++,j-=2)
				{
					CalTemp=(u16)(PortTemp<<16);
					*(pointer_2+i)=(u8)(CalTemp>>8);//先放高位
					*(pointer_2+i+1)=(u8)(CalTemp&0x00FF);//再放低位
				}
					
				//写入校验码						   
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断				
	
			}
			else
			{
				 ErrorHandle(3,pointer_2);//错误读取数量
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


  /* 函数功能：强制单个线圈状态
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;//待强制线圈起始地址（GPIO_X,X为A，B两个端口，每个端口16位，对应地址0——31）
	u16 Command=0;//命令为0xFF00或者0x0000，0xFF00强制线圈ON，0x0000强制线圈OFF；											
	u32 PortTemp;//用来存放从端口取过来的数据
	u32 CalTemp;//临时计算用
	u16 ReadData=0;//用来临时存放从端口读来的数据
	u16 SendKey;//要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到线圈地址
	Command=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));//先得到命令值
	*(pointer_2+2)=0x05;                               //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{

		if(Address<32) //只要地址小于32，就是合法地址
		{
		
			*(pointer_2)=1+1+2+2+2;//有效字节个数等于丛机地址+功能码+线圈地址+线圈命令+CRC校验
			*(pointer_2+3)=*(pointer_1+3);//将地址值写入输出的寄存器中
			*(pointer_2+4)=*(pointer_1+4);
	
			//将端口C和D的数据预先读入到临时的数据缓存中
			ReadData=GPIO_ReadOutputData(GPIOD);
			PortTemp=(u32)(ReadData);
			PortTemp=PortTemp<<16;
			ReadData=GPIO_ReadOutputData(GPIOC);
			PortTemp=PortTemp|(u32)(ReadData);
	
			if(Command==0xFF00||Command==0x0000)
			{
				//将PortTemp中的数据处理成需要的位数，再装入输出数据缓存中
				switch(Command)
				{
					case 0xFF00:
					{
						//将需要的一位置一
						CalTemp=0x00000001<<Address;
						PortTemp|=CalTemp;
	
						//将信息帧中命令对应位写上对应命令
						*(pointer_2+5)=0xFF;
						*(pointer_2+6)=0x00;
					}
					break;
	
					case 0x0000:
					{
						//将需要的位置零
						CalTemp=~(0x00000001<<Address);
						PortTemp&=CalTemp;
	
						//将信息帧中命令对应位写上对应命令
						*(pointer_2+5)=0x00;
						*(pointer_2+6)=0x00;
					}
					break;

				}
	
			    //再将数据写入
			    GPIO_Write(GPIOC,(u16)(PortTemp&0x0000FFFF));
			    GPIO_Write(GPIOD,(u16)(PortTemp>>16));
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
			}
			else
			{
				Comu_Busy=0;
			}
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


   /* 函数功能：预制单个寄存器
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void PresetSingleReg(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;    //待预制寄存器的起始地址（GPIO_X,X为A，B两个端口，每个端口16位，对应地址0——31）
	u16 PresetValue=0;//预制数值											
	u16 SendKey;      //要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));    //先得到寄存器地址
	PresetValue=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));//先得到预制值
	*(pointer_2+2)=0x06;                                   //第三个字节为功能码

	if(*(pointer_1)==8)	  //如果接收到的字节数不是8个，就是一个错误帧
	{

		if(Address<100) //只要地址小于100，就是合法地址
		{
		
				*(pointer_2)=1+1+2+2+2;       //有效字节个数等于丛机地址+功能码+寄存器地址+寄存器数值+CRC校验
				*(pointer_2+3)=*(pointer_1+3);//将地址值写入输出的寄存器中
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//将数值写入输出寄存器中
				*(pointer_2+6)=*(pointer_1+6);
	
				HoldReg[Address]=PresetValue; //将预制值写入保持寄存器
	
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}

	}
	else
	{
		Comu_Busy=0;
	}
 }


   /* 函数功能：强制多个线圈状态
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void ForceMulCoil(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;   //待强制线圈起始地址（GPIO_X,X为C，D两个端口，每个端口16位，对应地址0——31）
	u16 Num=0;       //要强制的线圈个数
	u8  ByteCount;   //命令值的字节个数
	u32 CommandValue;//命令的数值，有效的每个位用来置位或者复位指定地址的线圈											
	u32 PortTemp;    //用来存放从端口取过来的数据
	u32 CalTemp;     //临时计算用
	u16 ReadData=0;  //用来临时存放从端口读来的数据
	u16 SendKey;     //要发送数据的校验值
	u8  CountTemp;   //计算实际需要的命令字节数
	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到线圈地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到线圈数量
	ByteCount= *(pointer_1+7);                         //表示命令值的字节数
	CountTemp=(u8)(Num/8);
	if(Num%8!=0)
	CountTemp++;
	*(pointer_2+2)=0x0F;                               //第三个字节为功能码
	
										   
	if((*(pointer_1)==9+ByteCount)&&ByteCount>0&&ByteCount<=4&&CountTemp==ByteCount)//如果接收到的字节数不是预定的个数，或者命令字节数超出允许范围就是一个错误帧
	 {
	
		if(Address<32) //只要地址小于32，就是合法地址
		{
			if(Address+Num<=32&&Num>0) //只要地址加数量大于0小于32，就是合法数量
			{
				//用于for循环
				u8 i;
				u8 j;	

				//把命令值做一些处理，存入CommandValue
				CommandValue=0;
				for(i=0,j=ByteCount;j>0;i++,j--)
				{
					CommandValue|=((u32)*(pointer_1+8+i))<<8*i;//将输入数据缓存中的数据存入CommandValue
				}
				CommandValue=CommandValue<<Address;//移动到对应起始位位置

				*(pointer_2)=1+1+2+2+2;//有效字节个数等于丛机地址+功能码+线圈起始地址+线圈数量+CRC校验
				*(pointer_2+3)=*(pointer_1+3);//将地址值写入输出的寄存器中
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//将线圈数量值写入输出的寄存器中
				*(pointer_2+6)=*(pointer_1+6);
	
				//将端口C和D的数据预先读入到临时的数据缓存中
				ReadData=GPIO_ReadOutputData(GPIOD);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadOutputData(GPIOC);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//将需要读出来的数据按要求处理
				CalTemp=0xFFFFFFFF<<32-Num;
				CalTemp=CalTemp>>32-Num-Address;
				CalTemp=~CalTemp;
				PortTemp&=CalTemp;
				PortTemp|=CommandValue;
	
			    //再将数据写入
			    GPIO_Write(GPIOC,(u16)(PortTemp&0x0000FFFF));
			    GPIO_Write(GPIOD,(u16)(PortTemp>>16));
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
	
			}
		   else
		   {
		   		ErrorHandle(3,pointer_2);//错误读取数量
		   }
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
 	else
	{
		Comu_Busy=0;
	}
 }


    /* 函数功能：预制多个寄存器
 	函数输入：两个指针，pointer_1指向用来存放输入信息帧的数组，
			  pointer_2用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
			  后面的元素按照Modbus协议组织。														    
	函数输出：无。                                                                                      */

 void PresetMulReg(u8 *pointer_1,u8 *pointer_2)//pointer_1用作输入，pointer_2用作输出
 {
 	u16 Address=0;//待预制寄存器的起始地址（HoldReg[i],i为0-99对应地址从0到99）
	u16 Num=0;//要预制的寄存器数量
	u8  ByteCount;//预制值的字节个数
	u16 PresetValue=0;//预制数值											
	u16 SendKey;//要发送数据的校验值

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//先得到寄存器地址
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //先得到待预制寄存器数量
	*(pointer_2+2)=0x10;                               //第三个字节为功能码
	ByteCount= *(pointer_1+7);                         //表示命令值的字节数

	if((*(pointer_1)==9+ByteCount)&&ByteCount>0&&ByteCount<=200&&ByteCount==(u8)(Num*2))//如果接收到的字节数不是预定的个数，或者命令字节数超出允许范围就是一个错误帧
	{

		if(Address<100) //只要地址小于100，就是合法地址
		{
		
			if(Address+Num<=100&&Num>0) //只要地址加数量大于0小于100，就是合法数量
			{
				//用于for循环
				u8 i;
				u8 j;
					
				*(pointer_2)=1+1+2+2+2;       //有效字节个数等于丛机地址+功能码+寄存器地址+寄存器数量+CRC校验
				*(pointer_2+3)=*(pointer_1+3);//将地址值写入输出的寄存器中
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//将数量写入输出寄存器中
				*(pointer_2+6)=*(pointer_1+6);
	
				for(i=0,j=0;i<Num;i++,j+=2)		 	
				{
					PresetValue=(u16)(*(pointer_1+8+j))*256+(*(pointer_1+9+j));//先得到预制值
				 	HoldReg[Address+i]=PresetValue;                            //将预制值写入保持寄存器
				}
				
	
				//写入校验码
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//将计算出来的校验码装入输出数据缓存中
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//启动数据发送
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断
	
			}
			else
			{
				 ErrorHandle(3,pointer_2);//错误读取数量
			}
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//错误起始地址
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


													 
/* 函数功能：错误帧处理（处理1,2,3,6四类错误，其中1为不合法功能码，2不合法数据地址，3不合法数据，6从机设备忙碌）
函数输入：第一个参数Mode用来指示哪一类错误，
		  pointer用来指向存放输出信息帧的数组（两个数组的第一个元素都用来存放信息帧的有效字节个数）
		  后面的元素按照Modbus协议组织。														    
函数输出：无。                                                                                      */
 void  ErrorHandle(u8 Mode,u8 *Pointer)
 {
 	  u16 SendKey;//要发送数据的校验值

	  HaveMes=0;//清除信息位
	  TIM_Cmd(TIM2,DISABLE);
	  TIM_SetCounter(TIM3,0);
	  Rcv_Complete=1;
	  Comu_Busy=1;
	  Rcv_Buffer[0]=Rcv_Num;
 	  switch(Mode)
	  {
	  	case 1:*(Pointer+3)=0x01;//错误功能码
			break;			 
		case 2:*(Pointer+3)=0x02;//错误地址
			break;
		case 3:*(Pointer+3)=0x03;//错误数据
			break;
		case 6:*(Pointer+3)=0x06;//从设备忙
			break;
	  }
	  *Pointer=0x05;//输出寄存器有效数据个数	
	  *(Pointer+2)|=0x80;//功能码最高位置一
	  //写入校验码
	  SendKey=CRC16(Pointer+1,*Pointer-2);					
	  //将计算出来的校验码装入输出数据缓存中
	  *(Pointer+(*Pointer-1))=(u8)(SendKey>>8);
	  *(Pointer+(*Pointer))=(u8)(SendKey&0x00FF);
	
	  //启动数据发送
	  USART_ITConfig(USART2, USART_IT_TC, ENABLE);//开启数据发送中断	  																	   
 }
