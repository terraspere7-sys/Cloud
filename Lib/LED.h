
#ifndef _LED_H
#define _LED_H

#include "stm32f10x.h"

// LED 引脚定义 
#define LED1 GPIO_Pin_10
#define LED2 GPIO_Pin_11
#define LED3 GPIO_Pin_12

// LED 模块对外提供的函数 
void LED_Init(void);
void LED_On(uint16_t pin);
void LED_Off(uint16_t pin);
void LED_Toggle(uint16_t pin);
//三个函数分别控制LED的点亮/熄灭
void self_check(void);
//轮番点亮自检

#endif
