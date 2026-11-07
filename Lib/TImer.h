
#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

void TIM3_init(void);			//初始化TIM3用作全局计时器
uint32_t GetTick(void);		//获取当前时间
void Delay(uint32_t t);		//阻塞式延时函数
void Timer_Tick(void);  //供main.c中的中断函数调用计时

#endif
