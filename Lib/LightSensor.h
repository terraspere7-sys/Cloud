
#ifndef __LIGHTSENSOR_H
#define __LIGHTSENSOR_H

#include "stm32f10x.h"
#include <stdint.h>

/* ==================== 底层光敏传感器接口 ==================== */

/* 初始化 ADC1 和 PA0~PA3 */
void LightSensor_Init(void);

/* 读取指定通道：0~3 */
uint16_t LightSensor_Read(uint8_t channel);

/* 一次读取四路传感器，数组顺序为 L1、L2、L3、L4 */
void LightSensor_ReadAll(uint16_t value[4]);

/* ==================== 光线追踪状态 ==================== */

typedef struct
{
    uint16_t light[4];

    uint32_t avg_adc;
    uint16_t dark_threshold;

    int32_t EX;
    int32_t EY;
    int32_t FX;
    int32_t FY;

    uint8_t weak_light;

    uint8_t yaw_angle;
    uint8_t pitch_angle;
} LightTrack_Status;

/* ==================== 上层追光接口 ==================== */

/* 初始化追光状态并让两个舵机回到90° */
void LightTrack_Init(void);

/* 舵机回中、等待稳定、重新校准环境基准和弱光阈值 */
void LightTrack_Calibrate(void);

/* 更新一次传感器数据和追光控制，建议约20ms调用一次 */
void LightTrack_Update(void);

/* 获取最近一次更新后的状态 */
const LightTrack_Status *LightTrack_GetStatus(void);

#endif
