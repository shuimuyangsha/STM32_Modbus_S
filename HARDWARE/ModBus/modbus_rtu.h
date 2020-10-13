#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V1.1修改说明
//修改按键扫描函数，使整个代码可以支持SWD下载。
//////////////////////////////////////////////////////////////////////////////////	 
//声明modbus的函数
void ModInit(u8 Id);//用于Modbus初始化，参数Id为站号（1-255）
void ModRcv(void);//用于modbus信息接收
void ModSend(void);//用于modbus信息接收
void MessageHandle(u8 *pointer_in,u8 *pointer_out);//处理收到的信息帧
void ReadOutputBit(u8 *pointer_1,u8 *pointer_2);//读线圈
void ReadInputBit(u8 *pointer_1,u8 *pointer_2);//读输入位
void ReadHoldingReg(u8 *pointer_1,u8 *pointer_2);//读保持寄存器	
void ReadInputReg(u8 *pointer_1,u8 *pointer_2);//读输入寄存器
void ForceSingleCoil(u8 *pointer_1,u8 *pointer_2);//强制单个线圈
void PresetSingleReg(u8 *pointer_1,u8 *pointer_2);//预制单个寄存器
void ForceMulCoil(u8 *pointer_1,u8 *pointer_2);//强制多个线圈
void PresetMulReg(u8 *pointer_1,u8 *pointer_2);//预制多个寄存器
void ErrorHandle(u8 Mode,u8 *Pointer);//错误信息帧处理
u16 CRC16(u8 *puchMsgg,u8 usDataLen);//用于计算CRC校验码	

#endif


