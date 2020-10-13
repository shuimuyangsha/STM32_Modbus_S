#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V1.1�޸�˵��
//�޸İ���ɨ�躯����ʹ�����������֧��SWD���ء�
//////////////////////////////////////////////////////////////////////////////////	 
//����modbus�ĺ���
void ModInit(u8 Id);//����Modbus��ʼ��������IdΪվ�ţ�1-255��
void ModRcv(void);//����modbus��Ϣ����
void ModSend(void);//����modbus��Ϣ����
void MessageHandle(u8 *pointer_in,u8 *pointer_out);//�����յ�����Ϣ֡
void ReadOutputBit(u8 *pointer_1,u8 *pointer_2);//����Ȧ
void ReadInputBit(u8 *pointer_1,u8 *pointer_2);//������λ
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2);//�����ּĴ���	
void ReadInputReg(u8 *pointer_1,u8 *pointer_2);//������Ĵ���
void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2);//ǿ�Ƶ�����Ȧ
void PresetSingleReg(u8 *pointer_1,u8 *pointer_2);//Ԥ�Ƶ����Ĵ���
void ForceMulCoil(u8 *pointer_1,u8 *pointer_2);//ǿ�ƶ����Ȧ
void PresetMulReg(u8 *pointer_1,u8 *pointer_2);//Ԥ�ƶ���Ĵ���
void ErrorHandle(u8 Mode,u8 *Pointer);//������Ϣ֡����
u16 CRC16(u8 *puchMsgg,u8 usDataLen);//���ڼ���CRCУ����	

#endif


