#ifndef _MPU_6050_H
#define _MPU_6050_H

#include "stm32f10x.h"

/* 零位校准状态 */
#define MPU_CAL_IDLE       0
#define MPU_CAL_RUNNING    1
#define MPU_CAL_SUCCESS    2
#define MPU_CAL_FAILED     3

/* 非阻塞校准接口 */
uint8_t MPU_CalibrateStart(void);
void MPU_CalibrateTask(uint16_t sample);//sample是样本数量，每采集一个样本大约需要0.005秒
uint8_t MPU_CalibrateResult(void);
uint8_t MPU_ConnectCheck(void);


float get_yaw(void);
float get_pitch(void);
float get_roll(void);
void MPU_INIT(void);
void Data_Process(void);
void Gimbal_ControlTask(void);

#endif
