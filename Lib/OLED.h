#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"
#include <stdint.h>

/* 初始化：1=成功，0=失败 */
uint8_t OLED_Init(void);

/* 检查 OLED 在线状态：1=在线且地址有 ACK，0=异常 */
uint8_t OLED_CheckConnection(void);

/* 最近一次 OLED 写操作是否发生 ACK 错误：1=错误，0=正常 */
uint8_t OLED_HasError(void);

/* 清除整个屏幕 */
void OLED_Clear(void);

/* 清除指定页，page 范围为 0~7 */
void OLED_ClearLine(uint8_t page);

/* 设置显示位置，page 为 0~7，column 为 0~127 */
void OLED_SetPos(uint8_t page, uint8_t column);

/* 显示单个字符，支持数字和英文字母 */
void OLED_ShowChar(uint8_t page, uint8_t column, char ch);

/* 显示字符串 */
void OLED_ShowString(uint8_t page, uint8_t column, const char *str);

/* 显示一位数字，num 范围为 0~9 */
void OLED_ShowNum(uint8_t page, uint8_t column, uint8_t num);

#endif
