
#ifndef __SI2C_H
#define __SI2C_H

#include "stm32f10x.h"

#define SI2C_SCL GPIO_Pin_4
#define SI2C_SDA GPIO_Pin_5

/* 软件 I2C 初始化 */
void MY_SI2C_INIT(void);

/* GPIO 电平操作 */
void scl_write(uint8_t level);
void sda_write(uint8_t level);
uint8_t sda_read(void);

/* 微秒级延时 */
void delay_us(uint32_t t);

/* I2C 起始与停止信号 */
void sendstart(void);
void sendstop(void);

/* 单字节收发 */
uint8_t sendbyte(uint8_t byte);
uint8_t receivebyte(uint8_t ack);

#endif
