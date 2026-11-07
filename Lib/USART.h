
#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdarg.h>
#define RX_BUF_SIZE 128

/* 串口接收环形缓冲区 */
extern volatile uint8_t rx_buffer[RX_BUF_SIZE];
extern volatile uint16_t rx_read;
extern volatile uint16_t rx_write;
extern volatile uint8_t rx_overflow;

/* 串口函数 */
void USART_INIT(void);
void USART1_SendByte(uint8_t data);
void USART1_SendString(char *str);
void USART1_Printf(const char *Format, ...);

#endif
