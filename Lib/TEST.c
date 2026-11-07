
#include "stm32f10x.h"
#include "LightSensor.h"
#include "Timer.h"
#include "USART.h"
#include "Servo.h"
#include <stdint.h>
#include <stdio.h>

/* ==================== 参数配置 ==================== */

#define CAL_SAMPLE_INTERVAL  20U
#define CAL_SAMPLE_COUNT     100U
#define SAMPLE_INTERVAL      20U
#define PRINT_INTERVAL       200U

/* 水平、竖直方向分别设置死区 */
#define X_DEADBAND           100
#define Y_DEADBAND           100

/* ADC 越大表示越暗；弱光时暂停追踪 */
#define DARK_THRESHOLD       3000U

/* 滤波系数：每次修正误差的 1/4 */
#define FILTER_DIVISOR       5

/* 舵机参数 */
#define SERVO_CENTER         90.0f
#define SERVO_MIN_ANGLE       0.0f
#define SERVO_MAX_ANGLE     180.0f

/* 每秒最多改变的舵机角度 */
#define SERVO_SPEED          60.0f

/* ==================== 追踪状态变量 ==================== */

/* 校准基准 */
static int32_t dx_base = 0;
static int32_t dy_base = 0;

/* 原始误差与滤波误差 */
static int32_t EX = 0;
static int32_t EY = 0;
static int32_t FX = 0;
static int32_t FY = 0;

/* 舵机目标角度 */
static float yaw_angle = SERVO_CENTER;
static float pitch_angle = SERVO_CENTER;

/* ==================== 基础辅助函数 ==================== */

/* 使用现有 TIM3 毫秒计时 */
static void WaitMs(uint32_t ms)
{
    uint32_t start = GetTick();

    while ((uint32_t)(GetTick() - start) < ms)
    {
    }
}

/* 限制舵机角度在 0~180 度 */
static float Angle_Limit(float angle)
{
    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;

    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    return angle;
}

/* ==================== 光敏误差计算 ==================== */

/*
 * 传感器布局：
 *
 *       L4（左上）       L1（右上）
 *
 *       L3（左下）       L2（右下）
 *
 * light[0] = L1，light[1] = L2
 * light[2] = L3，light[3] = L4
 *
 * ADC 数值越大，代表该传感器接收到的光越弱。
 */
static void Light_CalculateError(
    uint16_t light[4],
    int32_t *dx,
    int32_t *dy)
{
    /* 左侧与右侧的亮度差 */
    *dx = (int32_t)light[3] + (int32_t)light[2]
        - (int32_t)light[0] - (int32_t)light[1];

    /* 下侧与上侧的亮度差 */
    *dy = (int32_t)light[2] + (int32_t)light[1]
        - (int32_t)light[3] - (int32_t)light[0];
}

/* ==================== 启动基准校准 ==================== */

/*
 * 上电后保持传感器板和光源不动。
 * 采集 100 组数据，每组间隔 20ms。
 */
static void Light_Calibrate(void)
{
    uint32_t i;
    int64_t dx_sum = 0;
    int64_t dy_sum = 0;
    uint16_t light[4];

    USART1_SendString(
        "\r\nLight calibration started.\r\n"
        "Keep sensor board and lamp still.\r\n");

    for (i = 0; i < CAL_SAMPLE_COUNT; i++)
    {
        LightSensor_ReadAll(light);

        {
            int32_t dx;
            int32_t dy;

            Light_CalculateError(light, &dx, &dy);

            dx_sum += dx;
            dy_sum += dy;
        }

        WaitMs(CAL_SAMPLE_INTERVAL);
    }

    dx_base = (int32_t)(dx_sum / CAL_SAMPLE_COUNT);
    dy_base = (int32_t)(dy_sum / CAL_SAMPLE_COUNT);

    EX = 0;
    EY = 0;
    FX = 0;
    FY = 0;

    USART1_Printf(
        "Calibration OK: DxBase=%ld DyBase=%ld\r\n",
        (long)dx_base,
        (long)dy_base);
}

/* ==================== 舵机追光控制 ==================== */

/*
 * 每 SAMPLE_INTERVAL 调用一次。
 *
 * EX > 0：右侧相对更亮，Yaw 增大，向右。
 * EX < 0：左侧相对更亮，Yaw 减小，向左。
 *
 * EY > 0：上侧相对更亮，Pitch 减小，向上。
 * EY < 0：下侧相对更亮，Pitch 增大，向下。
 */
static void LightServo_Control(
    int32_t fx,
    int32_t fy,
    uint8_t weak_light)
{
    float step;

    /* 弱光时不改变当前舵机角度 */
    if (weak_light)
        return;

    /* 20ms 对应的最大角度变化 */
    step = SERVO_SPEED * ((float)SAMPLE_INTERVAL / 1000.0f);

    /* -------- Yaw：水平追光 -------- */

    if (fx > X_DEADBAND)
    {
        yaw_angle += step;
    }
    else if (fx < -X_DEADBAND)
    {
        yaw_angle -= step;
    }

    /* -------- Pitch：竖直追光 -------- */

    if (fy > Y_DEADBAND)
    {
        pitch_angle -= step;
    }
    else if (fy < -Y_DEADBAND)
    {
        pitch_angle += step;
    }

    yaw_angle = Angle_Limit(yaw_angle);
    pitch_angle = Angle_Limit(pitch_angle);

    /* CH1 / PB6 = Yaw；CH2 / PB7 = Pitch */
    Servo_SetAngle1((uint8_t)(yaw_angle + 0.5f));
    Servo_SetAngle2((uint8_t)(pitch_angle + 0.5f));
}

/* ==================== 主测试函数 ==================== */

void TEST(void)
{
    uint16_t light[4];
    uint32_t last_print;
    uint32_t avg_adc;
    uint8_t weak_light;

    int32_t dx;
    int32_t dy;

    /* 外设初始化 */
    USART_INIT();
    TIM3_init();
    LightSensor_Init();
    Servo_Init();

    /* 舵机回中 */
    yaw_angle = SERVO_CENTER;
    pitch_angle = SERVO_CENTER;

    Servo_SetAngle1(90);
    Servo_SetAngle2(90);

    WaitMs(500);

    /* 启动时进行环境基准校准 */
    Light_Calibrate();

    last_print = GetTick();

    USART1_SendString(
        "Light tracking started.\r\n");

    while (1)
    {
        /* 1. 读取四路光敏 ADC */
        LightSensor_ReadAll(light);

        /* 2. 计算水平、竖直误差 */
        Light_CalculateError(light, &dx, &dy);

        /* 3. 扣除启动校准基准 */
        EX = dx - dx_base;
        EY = dy - dy_base;

        /* 4. 一阶整数滤波 */
        FX += (EX - FX) / FILTER_DIVISOR;
        FY += (EY - FY) / FILTER_DIVISOR;

        /* 5. 计算四路 ADC 平均值 */
        avg_adc = ((uint32_t)light[0]
                 + (uint32_t)light[1]
                 + (uint32_t)light[2]
                 + (uint32_t)light[3]) / 4U;

        /* 6. 判断是否处于弱光环境 */
        weak_light =
            (avg_adc >= DARK_THRESHOLD) ? 1U : 0U;

        /* 7. 更新舵机追光角度 */
        LightServo_Control(FX, FY, weak_light);

        /* 8. 每200ms打印一次调试信息 */
        if ((uint32_t)(GetTick() - last_print)
            >= PRINT_INTERVAL)
        {
            last_print = GetTick();

            USART1_Printf(
                "L1=%u L2=%u L3=%u L4=%u "
                "AVG=%lu EX=%ld EY=%ld FX=%ld FY=%ld "
                "%s X:%s Y:%s Yaw=%u Pitch=%u\r\n",
                light[0],
                light[1],
                light[2],
                light[3],
                (unsigned long)avg_adc,
                (long)EX,
                (long)EY,
                (long)FX,
                (long)FY,
                weak_light ? "WEAK" : "OK",
                (FX > X_DEADBAND) ? "RIGHT_DARK" :
                (FX < -X_DEADBAND) ? "LEFT_DARK" : "HOLD",
                (FY > Y_DEADBAND) ? "DOWN_DARK" :
                (FY < -Y_DEADBAND) ? "UP_DARK" : "HOLD",
                (unsigned int)(yaw_angle + 0.5f),
                (unsigned int)(pitch_angle + 0.5f));
        }

        /* 保持采样与舵机控制周期 */
        WaitMs(SAMPLE_INTERVAL);
    }
}
