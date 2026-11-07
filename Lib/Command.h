#ifndef __COMMAND_H
#define __COMMAND_H

#include "stm32f10x.h"

extern uint8_t oled_ok;
extern char oled_cmd_status[8];
extern uint8_t yaw_target_angle;
extern uint8_t pitch_target_angle;

void Cmd_Process(void);
void ToUpperString(char *str);//转换大小写（统一转到大写）
void USART_ParseCommand(char *cmd);//指令的判断
void OLED_ShowAxisTargets(void);

#endif
