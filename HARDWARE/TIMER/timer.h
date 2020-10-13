#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void Timer3_Init(u16 arr,u16 psc);
void Timer3_enable(void);
void Timer3_disable(void);

void Timer2_Init(u16 arr ,u16 psc);
void Timer2_enable(void);
void Timer2_disable(void);

void Timer4_enable(u16 arr,u16 psc);
void Timer4_disable(void);

extern u8 modbus_com_over;
extern u8 modbus_com2_over;

#endif
