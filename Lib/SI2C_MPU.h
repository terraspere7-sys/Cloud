
#ifndef __SI2C_MPU_H
#define __SI2C_MPU_H

#include "stm32f10x.h"

#define SI2C_SCL_mpu GPIO_Pin_6
#define SI2C_SDA_mpu GPIO_Pin_7

/* 软件 I2C 初始化 */
void MY_SI2C_MPU_INIT(void);

/* GPIO 电平操作 */
void scl_write_mpu(uint8_t level);
void sda_write_mpu(uint8_t level);
uint8_t sda_read_mpu(void);

/* 微秒级延时 */
void delay_us_mpu(uint32_t t);

/* I2C 起始与停止信号 */
void sendstart_mpu(void);
void sendstop_mpu(void);

/* 单字节收发 */
uint8_t sendbyte_mpu(uint8_t byte);
uint8_t receivebyte_mpu(uint8_t ack);

#endif
