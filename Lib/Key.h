
#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

#define Button GPIO_Pin_8

typedef enum {  //用一个枚举来表示当前按钮状态
	Key_IDLE,//空闲
	Key_DeBounce,//按下消抖
	Key_Pressed,//按下阶段
	Key_Released//松开消抖与长短按判定阶段
}KeyState; 
typedef struct{
	KeyState State; // 存储当前按钮状态
	uint32_t pressed_tick;  // 存储按下按键时间点
	uint32_t released_tick;  //存储松开按键时间点
}Key_t; 



extern Key_t a;
extern Key_t *p;

extern volatile uint8_t button_event;

void Key_init(void);
void Key_Scan(void);

#endif
