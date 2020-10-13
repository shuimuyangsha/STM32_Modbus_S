#include "modbus_rtu.h"
#include "usart.h"
#include "timer.h"
/*  ��ModbusЭ����ʱֻ֧��RTUģʽ��ֻ֧����ΪModbus���豸��
	��ʱ֧�ֵĹ����루16���ƣ����±���ʾ��
	01.����Ȧ״̬����������λ��״̬����Ч��λΪ0-31��
	02.������λ״̬�����������λ��״̬����Ч��λΪ0-31��
	03.�����ּĴ�������������ּĴ�������ֵ����Ч��λΪ0-99��
	04.������Ĵ��������������Ĵ�������ֵ����Ч��ַΪ0-1��
	05.ǿ�Ƶ�����Ȧ��ǿ�Ƶ������λ��״̬����Ч��λΪ0-31��
	06.Ԥ�Ƶ����Ĵ������趨һ���Ĵ�������ֵ����Ч��ַΪ0-99��
	0F.ǿ�ƶ����Ȧ��ǿ�ƶ�����λ��״̬����Ч��ַΪ0-31��
	10.Ԥ�ƶ���Ĵ������趨����Ĵ�������ֵ����Ч��ַΪ0-99��

	��ʱ֧�ֵĴ������Ϊ��
	01 ���Ϸ����ܴ���ӻ����յ���һ�ֲ���ִ�й��ܴ��롣������ѯ����󣬸ô���ָʾ�޳����ܡ�����֧�ֵĹ��ܴ��룩
    02 ���Ϸ����ݵ�ַ���յ����ݵ�ַ���Ǵӻ�������ĵ�ַ������ʼ��ַ������Ч��Χ�ڣ�
    03 ���Ϸ����ݲ�ѯ��������ֵ�Ǵӻ��������ֵ��������ʼ��ַ�Ļ����ϣ���������ǲ��Ϸ��ģ�

	���û����õĺ����У�
	1.void ModInit(u8 Id);//����Modbus��ʼ�����ں�������ǰ�������ʼ������������Main������
	2.void ModRcv(void);  //����modbus��Ϣ���գ����ڴ��ڽ����ж�
	3.void ModSend(void); //����modbus��Ϣ����,���ڴ��ڷ����ж�
	 ���磺	void USART1_IRQHandler(void)   //USART1�ж�
	{

	  if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
		{
			void ModRcv(void);
			����
			����
			����
		}

	  if(USART_GetITStatus(USART1,USART_IT_TC)!=RESET)
		{
			void ModSend(void);//����modbus��Ϣ����
			����
			����
			����
		}
	  
	}																				 */



//modbus��ͨѶ����
u8 Tim_Out;         //����3.5���ַ�ʱ�䣬����ȡ3ms (������9600��ʱ���Լ2�㼸����)
u8 Rcv_Complete;    //һ֡�Ƿ��Ѿ��������
u8 Send_Complete;   //һ֡�Ƿ��Ѿ��������
u8 Com_busy;        //ͨѶ��æ����ʾ��һ֡��δ�������
u8 Rcv_Buffer[210]; //������Ž��յ���������һ֡����	����һ���ֽ�������Ž��յ�����Ч�ֽ�����Ҳ���������е���Ч�ֽ�����					  
u8 Send_Buffer[210];//������Ŵ����͵�������һ֡���ݣ���һ���ֽ�������Ŵ����͵���Ч�ֽ�����Ҳ���������е���Ч�ֽ�����
u8 Rcv_Data;        //������Ž��յ�һ���ֽ�
u8 Send_Data;       //�������Ҫ���͵�һ�ֽ�
u8 Mod_Id;          //������־��Ϊ��վ��վ��
u8 Rcv_Num;         //������ʾ���յ�һ֡����Ч�ֽ������ӹ����뵽CRCУ�飩
u8 Send_Num;        //������ʾ�����͵�һ֡���ֽ���																				
u8 *PointToRcvBuf;  //����ָ����յ����ݻ���
u8 *PointToSendBuf; //����ָ������͵����ݻ���
u8 Comu_Busy;       //������ʾ�ܷ������һ֡����
u8 HaveMes;         //��Ϣ����ָʾ��=1��ʾ���յ���Ϣ��
extern u16 HoldReg[100];


//CRCУ�����ò���
/* CRC ��λ�ֽ�ֵ��*/
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
/* CRC��λ�ֽ�ֵ��*/
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

//����modbus�ĺ���
void ModInit(u8 Id);                               //����Modbus��ʼ��������IdΪվ�ţ�1-255��
void ModRcv(void);                                 //����modbus��Ϣ����
void ModSend(void);                                //����modbus��Ϣ����
void MessageHandle(u8 *pointer_in,u8 *pointer_out);//�����յ�����Ϣ֡
void ReadOutputBit(u8 *pointer_1,u8 *pointer_2);   //����Ȧ
void ReadInputBit(u8 *pointer_1,u8 *pointer_2);    //������λ
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2);  //�����ּĴ���	
void ReadInputReg(u8 *pointer_1,u8 *pointer_2);    //������Ĵ���
void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2); //ǿ�Ƶ�����Ȧ
void PresetSingleReg(u8 *pointer_1,u8 *pointer_2); //Ԥ�Ƶ����Ĵ���
void ForceMulCoil(u8 *pointer_1,u8 *pointer_2);    //ǿ�ƶ����Ȧ
void PresetMulReg(u8 *pointer_1,u8 *pointer_2);    //Ԥ�ƶ���Ĵ���
void ErrorHandle(u8 Mode,u8 *Pointer);             //������Ϣ֡����
u16 CRC16(u8 *puchMsgg,u8 usDataLen);              //���ڼ���CRCУ����

 /* �������ܣ�����Modbus��ʼ��
 	�������룺IdΪModbusվ�š�												
	����������ޡ�                                                                                      */
  void ModInit(u8 Id)
  {
  	//modbus������ʼ��
	  PointToRcvBuf=Rcv_Buffer;
    PointToSendBuf=Send_Buffer;
    Send_Num=1;//���͵�����˳���������ĵڼ�������
	  Mod_Id=Id;//վ������
    Rcv_Buffer[1]=Mod_Id;
    Send_Buffer[1]=Mod_Id;
	  Comu_Busy=0;
  }
  
 /* �������ܣ�����Modbus��Ϣ����
 	�������룺�ޡ�														    
	����������ޡ�                                                                                      */
  void ModRcv(void)
  {
		HaveMes=1;//��ʾ���յ�����Ϣ
		Rcv_Data=USART_ReceiveData(USART2);
		if(Comu_Busy!=1)//�����æ�����Խ�����һ֡��Ϣ
		{
			TIM_Cmd(TIM2, DISABLE);//�ر�ʱ��
	    TIM_SetCounter(TIM2,0);//��ʼ��TIM2�������ĳ�ʼֵΪ0
			if((Tim_Out!=0)&&(Rcv_Data==Mod_Id))//������ʱ�䳬����3.5���ַ���ͬʱ���ܵ��ֽں��Լ���վ��һ�£�����Ϊ���տ�ʼ
			{
				Rcv_Complete=0; //��ʾ����֡���տ�ʼ
				Rcv_Num=0;      //�������ݸ�����ʼ��
				Rcv_Num++;      //ͬʱ������һ
			}
			if((0==Tim_Out)&&(0==Rcv_Complete))//������ڽ���һ֡������������
			{
				if(Rcv_Num<100)
				{
					Rcv_Buffer[Rcv_Num+1]=Rcv_Data;//�����ݷ������������
					Rcv_Num++;//ͬʱ������һ	
				}
				else
				{
					Rcv_Complete=1;
					Comu_Busy=1;
					Rcv_Buffer[0]=Rcv_Num;
					*(PointToSendBuf+2)=*(PointToRcvBuf+2);//��ȡ������
					ErrorHandle(6,PointToSendBuf);//��ʾ�������ֽ���(�ӻ��豸æµ)
					Rcv_Num=0;
				}
			}
			Tim_Out=0;
		  TIM_Cmd(TIM2, ENABLE);//����4.5ms��ʱ���������ַ��ı��ع��ƣ�
		}
  }

 /* �������ܣ�����Modbus��Ϣ����
 	�������룺�ޡ�														    
	����������ޡ�																	   */
  void ModSend(void)	
 {	
 	Send_Data=*(PointToSendBuf+Send_Num);
	USART_ClearFlag(USART2,USART_FLAG_TC);                 //��ȡUSART_SR
  USART_SendData(USART2,Send_Data);
	Send_Num++;
	if(Send_Num>(*PointToSendBuf))//�����Ѿ����
	{
		Comu_Busy=0;
		*PointToSendBuf=0;
		Rcv_Num=0;
		Send_Num=1;
		//�������ݷ���
	  USART_ITConfig(USART2, USART_IT_TC, DISABLE);//�ر����ݷ����ж�	
	}
 }
                                                                                     
 /* �������ܣ�Modbusר�ö�ʱ����TIM2��
 	�������룺�ޡ�														    
	����������ޡ�													  */
void TIM2_IRQHandler(void)															  
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ
	  Tim_Out=1;
		TIM_Cmd(TIM2,DISABLE);//�ر�ʱ��
		TIM_SetCounter(TIM2,0);//��ʼ��TIM2�������ĳ�ʼֵΪ0
		Rcv_Complete=1;
		Rcv_Buffer[0]=Rcv_Num;

		if(HaveMes!=0&&Rcv_Num>4)
		{
			Comu_Busy=1;
			MessageHandle(PointToRcvBuf,PointToSendBuf);
		}
		
	}	
}

	
	
	
 /* �������ܣ�CRCУ���ú���
 	�������룺puchMsgg��Ҫ����CRCУ�����Ϣ��usDataLen����Ϣ���ֽ���														    
	������������������CRCУ���롣                                                                                      */
u16 CRC16(u8 *puchMsgg,u8 usDataLen)//puchMsgg��Ҫ����CRCУ�����Ϣ��usDataLen����Ϣ���ֽ��� 
{
    u8 uchCRCHi = 0xFF ; /* ��CRC�ֽڳ�ʼ��*/
    u8 uchCRCLo = 0xFF ; /* ��CRC �ֽڳ�ʼ��*/
    u8 uIndex ; /* CRCѭ���е�����*/
    while (usDataLen--) /* ������Ϣ������*/
    {
				uIndex = uchCRCHi ^ *puchMsgg++ ; /* ����CRC */
				uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
				uchCRCLo = auchCRCLo[uIndex] ;
    }
    return ((uchCRCHi << 8) | uchCRCLo) ;
}

 /* �������ܣ����������Ϣ֡���д������չ����벻ͬ�����ò�ͬ�ĺ�������
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */
void MessageHandle(u8 *pointer_in,u8 *pointer_out)		  
 {
 	u16 CalKey;//���������У��ֵ
	u16 RcvKey;//���յ���У��ֵ
	
	HaveMes=0;//�����Ϣλ
	//��ȡ���յ���У��ֵ
	RcvKey=(u16)*(pointer_in+(*pointer_in-1));
	RcvKey=RcvKey<<8;
	RcvKey=RcvKey|(u16)*(pointer_in+(*pointer_in));

	CalKey=CRC16(pointer_in+1,*pointer_in-2);
	if(CalKey==RcvKey)
	{
		switch(*(pointer_in+2))//�ڶ����ֽ�Ϊ������
		{
			case 0x01:ReadOutputBit(pointer_in,pointer_out);  //�������Ȧ
				break;
			case 0x02:ReadInputBit(pointer_in,pointer_out);   //������λ
				break;
			case 0x03:ReadHoldingReg(pointer_in,pointer_out); //�����ּĴ���
				break;
			case 0x04:ReadInputReg(pointer_in,pointer_out);   //������Ĵ���
				break;
			case 0x05:ForceSingleCoil(pointer_in,pointer_out);//ǿ�Ƶ�����Ȧ״̬
				break;
			case 0x06:PresetSingleReg(pointer_in,pointer_out);//Ԥ�Ƶ����Ĵ���
				break;
			case 0x0F:ForceMulCoil(pointer_in,pointer_out);   //ǿ�ƶ����Ȧ
				break;
			case 0x10:PresetMulReg(pointer_in,pointer_out);   //Ԥ�ƶ���Ĵ���
				break;												 
			default: 
			{
				*(pointer_out+2)=*(pointer_in+2);//��ȡ������
				ErrorHandle(1,pointer_out);      //��������� 
			}	
				break;
		}
	}
	else
	{
		Comu_Busy=0;
	} 		
 }


  /* �������ܣ���ȡ��Ȧ״̬
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void ReadOutputBit(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0; //����ȡ��Ȧ��ʼ��ַ��GPIO_X,XΪA��B�����˿ڣ�ÿ���˿�16λ����Ӧ��ַ0����31��												
	u16 Num=0;     //Ҫ��ȡ����Ȧ����
	u8 Byte=0;     //Ҫ��ȡ����Ȧ�����ܹ�ռ�õ��ֽ�����
	u32 PortTemp;  //������ŴӶ˿�ȡ����������
	u16 ReadData=0;//������ʱ��ŴӶ˿ڶ���������
	u16 SendKey;   //Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ���Ȧ��ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ�Ҫ��ȡ����Ȧ����
	*(pointer_2+2)=0x01;                               //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{
		if(Address<32) //ֻҪ��ַС��32�����ǺϷ���ַ
		{
			if(Address+Num<=32&&Num>0) //ֻҪ��ַ����������0С��80�����ǺϷ�����
			{	
				//����forѭ��
				u8 i;
				u8 j;

				Byte=Num/8;
				if(Num%8!=0)
				Byte++;//����������Ļ�����һ���ֽڣ�ʣ��ĸ�λ����
				*(pointer_2+3)=Byte;//���ĸ��ֽ�ΪҪ���͵ĸ���
				*(pointer_2)=1+1+1+Byte+2;//��Ч�ֽڸ������ڴԻ���ַ+������+�ֽڸ���+��Ȧ��Ϣ+CRCУ��
	
				//���˿�C��D������Ԥ�ȶ��뵽��ʱ�����ݻ�����
				ReadData=GPIO_ReadOutputData(GPIOD);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadOutputData(GPIOC);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//��PortTemp�е����ݴ������Ҫ��λ������װ��������ݻ�����
				PortTemp=PortTemp<<(31-(Address+Num-1));
				PortTemp=PortTemp>>((31-(Address+Num-1))+Address);
	
				//������һ���ֽ�һ���ֽ�װ���������ݻ���������
				
				for(i=4,j=Byte;j>0;j--,i++)
				{
					*(pointer_2+i)=(u8)(PortTemp&0x00FF);//��ȡһ���ֽڵĳ��ȣ��������
					PortTemp=PortTemp>>8;//�ٽ����������ƶ�8λ
				}
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
									
			}
			else
			{
				 ErrorHandle(3,pointer_2);//�����ȡ����
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


 /* �������ܣ���ȡ����״̬
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void ReadInputBit(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;//����ȡ����λ��ʼ��ַ��GPIO_X,XΪC��D�����˿ڣ�ÿ���˿�16λ����Ӧ��ַ0-31��
	u16 Num=0;//Ҫ��ȡ������λ����
	u8 Byte=0;//Ҫ��ȡ������λ�����ܹ�ռ�õ��ֽ�����
	u32 PortTemp;//������ŴӶ˿�ȡ���������ݣ���ʱ������
	u16 ReadData=0;//������ʱ��ŴӶ˿ڶ���������
	u16 SendKey;//Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ�����λ��ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ�Ҫ��ȡ������λ����
	*(pointer_2+2)=0x02;                               //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{

		if(Address<32) //ֻҪ��ַС��32�����ǺϷ���ַ
		{
			if(Address+Num<=32&&Num>0) //ֻҪ��ַ����������0С��80�����ǺϷ�����
			{
				//����forѭ��
				u8 i;
				u8 j;
				
				Byte=Num/8;
				if(Num%8!=0)
				Byte++;//����������Ļ�����һ���ֽڣ�ʣ��ĸ�λ����
				*(pointer_2+3)=Byte;//���ĸ��ֽ�ΪҪ���͵ĸ���
				*(pointer_2)=1+1+1+Byte+2;//��Ч�ֽڸ������ڴԻ���ַ+������+�ֽڸ���+����λ��Ϣ+CRCУ��
	
				//���˿�A��B������Ԥ�ȶ��뵽��ʱ�����ݻ�����
				ReadData=GPIO_ReadInputData(GPIOB);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadInputData(GPIOA);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//��PortTemp�е����ݴ������Ҫ��λ������װ��������ݻ�����
				PortTemp=PortTemp<<(31-(Address+Num-1));
				PortTemp=PortTemp>>((31-(Address+Num-1))+Address);
	
				//������һ���ֽ�һ���ֽ�װ���������ݻ���������
				for(i=4,j=Byte;j>0;j--,i++)
				{
					*(pointer_2+i)=(u8)(PortTemp&0x00FF);//��ȡһ���ֽڵĳ��ȣ��������
					PortTemp=PortTemp>>8;//�ٽ����������ƶ�8λ
				}
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
				 
			}
			else
			{
				 ErrorHandle(3,pointer_2);//�����ȡ����
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


/* �������ܣ���ȡ���ּĴ���
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�
	                                                                                      */
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;//����ȡ�Ĵ�����ʼ��ַ��HoldReg[i],iΪ0-99��Ӧ��ַ��0��99��
	u16 Num=0;    //Ҫ��ȡ�ļĴ�������
	u16 SendKey;  //Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ��Ĵ�����ʼ��ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ�Ҫ��ȡ�ļĴ�������
	*(pointer_2+2)=0x03;                               //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{

		if(Address<100) //ֻҪ��ַС��100�����ǺϷ���ַ
		{
			if(Address+Num<=100&&Num>0) //ֻҪ��ַ����������0С��100�����ǺϷ�����
			{
				
 				//����forѭ��
				u8 i;
				u8 j;

				*(pointer_2+3)=Num*2;      //���ĸ��ֽ�ΪҪ���͵��ֽڸ���
				*(pointer_2)=1+1+1+Num*2+2;//��Ч�ֽڸ������ڴԻ���ַ+������+�ֽڸ���+�Ĵ�����Ϣ+CRCУ��
	
	
				for(i=Address,j=4;i<Address+Num;i++,j+=2)
				{
					*(pointer_2+j)=(u8)(HoldReg[i]>>8);//�ȷŸ�λ
					*(pointer_2+j+1)=(u8)(HoldReg[i]&0x00FF);//�ٷŵ�λ
				}
					
				//д��У����						   
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
						
			}
			else
			{
				 ErrorHandle(3,pointer_2);//�����ȡ����
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


/* �������ܣ���ȡ����Ĵ�����ģ�������룩
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�
	                                                                                      */
 void ReadInputReg(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;//����ȡ�Ĵ�����ʼ��ַ��HoldReg[i],iΪ0-99��Ӧ��ַ��0��99��
	u16 Num=0;//Ҫ��ȡ�ļĴ�������
	u16 SendKey;//Ҫ�������ݵ�У��ֵ
	u32 PortTemp;//������ŴӶ˿�ȡ���������ݣ���ʱ������
	u16 ReadData=0;//������ʱ��ŴӶ˿ڶ���������
	u32 CalTemp=0;//������ʱ����

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ��Ĵ�����ʼ��ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ�Ҫ��ȡ�ļĴ�������
	*(pointer_2+2)=0x04;                               //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{
		if(Address<2) //ֻҪ��ַС��2�����ǺϷ���ַ
		{
			if(Address+Num<=2&&Num>0) //ֻҪ��ַ����������0С��2�����ǺϷ�����
			{
				//����forѭ��
				u8 i;
				u8 j;
								
				*(pointer_2+3)=Num*2;//���ĸ��ֽ�ΪҪ���͵��ֽڸ���
				*(pointer_2)=1+1+1+Num*2+2;//��Ч�ֽڸ������ڴԻ���ַ+������+�ֽڸ���+�Ĵ�����Ϣ+CRCУ��
	
				//���˿�ADC1��ADC2������Ԥ�ȶ��뵽��ʱ�����ݻ�����
				ReadData=0x01;    //2014-9-28
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=0x02;    //2014-9-28
				PortTemp=PortTemp|(u32)(ReadData);
				
				//��PortTemp�е������Ƚ���Ԥ����
				PortTemp=PortTemp<<(Address*16);
				for(i=4,j=Num*2;j>0;i++,j-=2)
				{
					CalTemp=(u16)(PortTemp<<16);
					*(pointer_2+i)=(u8)(CalTemp>>8);//�ȷŸ�λ
					*(pointer_2+i+1)=(u8)(CalTemp&0x00FF);//�ٷŵ�λ
				}
					
				//д��У����						   
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�				
	
			}
			else
			{
				 ErrorHandle(3,pointer_2);//�����ȡ����
			}		
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


  /* �������ܣ�ǿ�Ƶ�����Ȧ״̬
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;//��ǿ����Ȧ��ʼ��ַ��GPIO_X,XΪA��B�����˿ڣ�ÿ���˿�16λ����Ӧ��ַ0����31��
	u16 Command=0;//����Ϊ0xFF00����0x0000��0xFF00ǿ����ȦON��0x0000ǿ����ȦOFF��											
	u32 PortTemp;//������ŴӶ˿�ȡ����������
	u32 CalTemp;//��ʱ������
	u16 ReadData=0;//������ʱ��ŴӶ˿ڶ���������
	u16 SendKey;//Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ���Ȧ��ַ
	Command=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));//�ȵõ�����ֵ
	*(pointer_2+2)=0x05;                               //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{

		if(Address<32) //ֻҪ��ַС��32�����ǺϷ���ַ
		{
		
			*(pointer_2)=1+1+2+2+2;//��Ч�ֽڸ������ڴԻ���ַ+������+��Ȧ��ַ+��Ȧ����+CRCУ��
			*(pointer_2+3)=*(pointer_1+3);//����ֵַд������ļĴ�����
			*(pointer_2+4)=*(pointer_1+4);
	
			//���˿�C��D������Ԥ�ȶ��뵽��ʱ�����ݻ�����
			ReadData=GPIO_ReadOutputData(GPIOD);
			PortTemp=(u32)(ReadData);
			PortTemp=PortTemp<<16;
			ReadData=GPIO_ReadOutputData(GPIOC);
			PortTemp=PortTemp|(u32)(ReadData);
	
			if(Command==0xFF00||Command==0x0000)
			{
				//��PortTemp�е����ݴ������Ҫ��λ������װ��������ݻ�����
				switch(Command)
				{
					case 0xFF00:
					{
						//����Ҫ��һλ��һ
						CalTemp=0x00000001<<Address;
						PortTemp|=CalTemp;
	
						//����Ϣ֡�������Ӧλд�϶�Ӧ����
						*(pointer_2+5)=0xFF;
						*(pointer_2+6)=0x00;
					}
					break;
	
					case 0x0000:
					{
						//����Ҫ��λ����
						CalTemp=~(0x00000001<<Address);
						PortTemp&=CalTemp;
	
						//����Ϣ֡�������Ӧλд�϶�Ӧ����
						*(pointer_2+5)=0x00;
						*(pointer_2+6)=0x00;
					}
					break;

				}
	
			    //�ٽ�����д��
			    GPIO_Write(GPIOC,(u16)(PortTemp&0x0000FFFF));
			    GPIO_Write(GPIOD,(u16)(PortTemp>>16));
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
			}
			else
			{
				Comu_Busy=0;
			}
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


   /* �������ܣ�Ԥ�Ƶ����Ĵ���
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void PresetSingleReg(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;    //��Ԥ�ƼĴ�������ʼ��ַ��GPIO_X,XΪA��B�����˿ڣ�ÿ���˿�16λ����Ӧ��ַ0����31��
	u16 PresetValue=0;//Ԥ����ֵ											
	u16 SendKey;      //Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));    //�ȵõ��Ĵ�����ַ
	PresetValue=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));//�ȵõ�Ԥ��ֵ
	*(pointer_2+2)=0x06;                                   //�������ֽ�Ϊ������

	if(*(pointer_1)==8)	  //������յ����ֽ�������8��������һ������֡
	{

		if(Address<100) //ֻҪ��ַС��100�����ǺϷ���ַ
		{
		
				*(pointer_2)=1+1+2+2+2;       //��Ч�ֽڸ������ڴԻ���ַ+������+�Ĵ�����ַ+�Ĵ�����ֵ+CRCУ��
				*(pointer_2+3)=*(pointer_1+3);//����ֵַд������ļĴ�����
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//����ֵд������Ĵ�����
				*(pointer_2+6)=*(pointer_1+6);
	
				HoldReg[Address]=PresetValue; //��Ԥ��ֵд�뱣�ּĴ���
	
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}

	}
	else
	{
		Comu_Busy=0;
	}
 }


   /* �������ܣ�ǿ�ƶ����Ȧ״̬
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void ForceMulCoil(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;   //��ǿ����Ȧ��ʼ��ַ��GPIO_X,XΪC��D�����˿ڣ�ÿ���˿�16λ����Ӧ��ַ0����31��
	u16 Num=0;       //Ҫǿ�Ƶ���Ȧ����
	u8  ByteCount;   //����ֵ���ֽڸ���
	u32 CommandValue;//�������ֵ����Ч��ÿ��λ������λ���߸�λָ����ַ����Ȧ											
	u32 PortTemp;    //������ŴӶ˿�ȡ����������
	u32 CalTemp;     //��ʱ������
	u16 ReadData=0;  //������ʱ��ŴӶ˿ڶ���������
	u16 SendKey;     //Ҫ�������ݵ�У��ֵ
	u8  CountTemp;   //����ʵ����Ҫ�������ֽ���
	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ���Ȧ��ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ���Ȧ����
	ByteCount= *(pointer_1+7);                         //��ʾ����ֵ���ֽ���
	CountTemp=(u8)(Num/8);
	if(Num%8!=0)
	CountTemp++;
	*(pointer_2+2)=0x0F;                               //�������ֽ�Ϊ������
	
										   
	if((*(pointer_1)==9+ByteCount)&&ByteCount>0&&ByteCount<=4&&CountTemp==ByteCount)//������յ����ֽ�������Ԥ���ĸ��������������ֽ�����������Χ����һ������֡
	 {
	
		if(Address<32) //ֻҪ��ַС��32�����ǺϷ���ַ
		{
			if(Address+Num<=32&&Num>0) //ֻҪ��ַ����������0С��32�����ǺϷ�����
			{
				//����forѭ��
				u8 i;
				u8 j;	

				//������ֵ��һЩ��������CommandValue
				CommandValue=0;
				for(i=0,j=ByteCount;j>0;i++,j--)
				{
					CommandValue|=((u32)*(pointer_1+8+i))<<8*i;//���������ݻ����е����ݴ���CommandValue
				}
				CommandValue=CommandValue<<Address;//�ƶ�����Ӧ��ʼλλ��

				*(pointer_2)=1+1+2+2+2;//��Ч�ֽڸ������ڴԻ���ַ+������+��Ȧ��ʼ��ַ+��Ȧ����+CRCУ��
				*(pointer_2+3)=*(pointer_1+3);//����ֵַд������ļĴ�����
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//����Ȧ����ֵд������ļĴ�����
				*(pointer_2+6)=*(pointer_1+6);
	
				//���˿�C��D������Ԥ�ȶ��뵽��ʱ�����ݻ�����
				ReadData=GPIO_ReadOutputData(GPIOD);
				PortTemp=(u32)(ReadData);
				PortTemp=PortTemp<<16;
				ReadData=GPIO_ReadOutputData(GPIOC);
				PortTemp=PortTemp|(u32)(ReadData);
	
				//����Ҫ�����������ݰ�Ҫ����
				CalTemp=0xFFFFFFFF<<32-Num;
				CalTemp=CalTemp>>32-Num-Address;
				CalTemp=~CalTemp;
				PortTemp&=CalTemp;
				PortTemp|=CommandValue;
	
			    //�ٽ�����д��
			    GPIO_Write(GPIOC,(u16)(PortTemp&0x0000FFFF));
			    GPIO_Write(GPIOD,(u16)(PortTemp>>16));
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
	
			}
		   else
		   {
		   		ErrorHandle(3,pointer_2);//�����ȡ����
		   }
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
 	else
	{
		Comu_Busy=0;
	}
 }


    /* �������ܣ�Ԥ�ƶ���Ĵ���
 	�������룺����ָ�룬pointer_1ָ���������������Ϣ֡�����飬
			  pointer_2����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
			  �����Ԫ�ذ���ModbusЭ����֯��														    
	����������ޡ�                                                                                      */

 void PresetMulReg(u8 *pointer_1,u8 *pointer_2)//pointer_1�������룬pointer_2�������
 {
 	u16 Address=0;//��Ԥ�ƼĴ�������ʼ��ַ��HoldReg[i],iΪ0-99��Ӧ��ַ��0��99��
	u16 Num=0;//ҪԤ�ƵļĴ�������
	u8  ByteCount;//Ԥ��ֵ���ֽڸ���
	u16 PresetValue=0;//Ԥ����ֵ											
	u16 SendKey;//Ҫ�������ݵ�У��ֵ

	
	Address=(u16)(*(pointer_1+3))*256+(*(pointer_1+4));//�ȵõ��Ĵ�����ַ
	Num=(u16)(*(pointer_1+5))*256+(*(pointer_1+6));    //�ȵõ���Ԥ�ƼĴ�������
	*(pointer_2+2)=0x10;                               //�������ֽ�Ϊ������
	ByteCount= *(pointer_1+7);                         //��ʾ����ֵ���ֽ���

	if((*(pointer_1)==9+ByteCount)&&ByteCount>0&&ByteCount<=200&&ByteCount==(u8)(Num*2))//������յ����ֽ�������Ԥ���ĸ��������������ֽ�����������Χ����һ������֡
	{

		if(Address<100) //ֻҪ��ַС��100�����ǺϷ���ַ
		{
		
			if(Address+Num<=100&&Num>0) //ֻҪ��ַ����������0С��100�����ǺϷ�����
			{
				//����forѭ��
				u8 i;
				u8 j;
					
				*(pointer_2)=1+1+2+2+2;       //��Ч�ֽڸ������ڴԻ���ַ+������+�Ĵ�����ַ+�Ĵ�������+CRCУ��
				*(pointer_2+3)=*(pointer_1+3);//����ֵַд������ļĴ�����
				*(pointer_2+4)=*(pointer_1+4);
				*(pointer_2+5)=*(pointer_1+5);//������д������Ĵ�����
				*(pointer_2+6)=*(pointer_1+6);
	
				for(i=0,j=0;i<Num;i++,j+=2)		 	
				{
					PresetValue=(u16)(*(pointer_1+8+j))*256+(*(pointer_1+9+j));//�ȵõ�Ԥ��ֵ
				 	HoldReg[Address+i]=PresetValue;                            //��Ԥ��ֵд�뱣�ּĴ���
				}
				
	
				//д��У����
				SendKey=CRC16(pointer_2+1,*pointer_2-2);					
				//�����������У����װ��������ݻ�����
				*(pointer_2+(*pointer_2-1))=(u8)(SendKey>>8);
				*(pointer_2+(*pointer_2))=(u8)(SendKey&0x00FF);
	
				//�������ݷ���
				USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�
	
			}
			else
			{
				 ErrorHandle(3,pointer_2);//�����ȡ����
			}
	
		}
		else
		{
			ErrorHandle(2,pointer_2);//������ʼ��ַ
		}
	}
	else
	{
		Comu_Busy=0;
	}
 }


													 
/* �������ܣ�����֡��������1,2,3,6�����������1Ϊ���Ϸ������룬2���Ϸ����ݵ�ַ��3���Ϸ����ݣ�6�ӻ��豸æµ��
�������룺��һ������Mode����ָʾ��һ�����
		  pointer����ָ���������Ϣ֡�����飨��������ĵ�һ��Ԫ�ض����������Ϣ֡����Ч�ֽڸ�����
		  �����Ԫ�ذ���ModbusЭ����֯��														    
����������ޡ�                                                                                      */
 void  ErrorHandle(u8 Mode,u8 *Pointer)
 {
 	  u16 SendKey;//Ҫ�������ݵ�У��ֵ

	  HaveMes=0;//�����Ϣλ
	  TIM_Cmd(TIM2,DISABLE);
	  TIM_SetCounter(TIM3,0);
	  Rcv_Complete=1;
	  Comu_Busy=1;
	  Rcv_Buffer[0]=Rcv_Num;
 	  switch(Mode)
	  {
	  	case 1:*(Pointer+3)=0x01;//��������
			break;			 
		case 2:*(Pointer+3)=0x02;//�����ַ
			break;
		case 3:*(Pointer+3)=0x03;//��������
			break;
		case 6:*(Pointer+3)=0x06;//���豸æ
			break;
	  }
	  *Pointer=0x05;//����Ĵ�����Ч���ݸ���	
	  *(Pointer+2)|=0x80;//���������λ��һ
	  //д��У����
	  SendKey=CRC16(Pointer+1,*Pointer-2);					
	  //�����������У����װ��������ݻ�����
	  *(Pointer+(*Pointer-1))=(u8)(SendKey>>8);
	  *(Pointer+(*Pointer))=(u8)(SendKey&0x00FF);
	
	  //�������ݷ���
	  USART_ITConfig(USART2, USART_IT_TC, ENABLE);//�������ݷ����ж�	  																	   
 }
