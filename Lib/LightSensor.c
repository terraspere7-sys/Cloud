
#include "LightSensor.h"
#include "Timer.h"
#include "Servo.h"
#include <stdint.h>

/* =========================================================
 * 底层 ADC 参数
 * ========================================================= */

#define LIGHT_ADC_COUNT          4U
#define LIGHT_ADC_MAX_VALUE      4095U

/* ADC 采样时间 */
#define LIGHT_ADC_SAMPLE_TIME    ADC_SampleTime_55Cycles5

/* =========================================================
 * 追光参数
 * ========================================================= */

#define LT_CAL_SAMPLE_INTERVAL   20U //采样间隔
#define LT_CAL_SAMPLE_COUNT      100U//采样样本数
#define LT_SAMPLE_INTERVAL       20U//运行时的更新周期

#define LT_X_DEADBAND            100
#define LT_Y_DEADBAND            100
//水平和竖直方向的死区
#define LT_FILTER_DIVISOR        5
//滤波系数 α = 1/5
#define LT_SERVO_CENTER          90.0f
#define LT_SERVO_MIN_ANGLE        0.0f
#define LT_SERVO_MAX_ANGLE      180.0f
#define LT_SERVO_SPEED           60.0f//舵机的最大角速度

/* 校准平均 ADC 值减去400 */
#define LT_DARK_MARGIN           400U//(校准均值-400)作为弱光阈值

/* =========================================================
 * 底层 ADC 初始化与读取
 * ========================================================= */

/*
 * PA0 -> ADC1_IN0 -> L1
 * PA1 -> ADC1_IN1 -> L2
 * PA2 -> ADC1_IN2 -> L3
 * PA3 -> ADC1_IN3 -> L4
 *
 * 光敏模块应使用3.3V供电，确保ADC引脚电压不超过VDDA。
 */

void LightSensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    /* 开启 GPIOA 和 ADC1 时钟 */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1,
        ENABLE);

    /* STM32F103 ADC 时钟最大14MHz。
       PCLK2=72MHz时，ADC预分频选择6，得到12MHz。 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PA0~PA3 配置为模拟输入 */
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ADC1 基础配置 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv =
        ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    ADC_Init(ADC1, &ADC_InitStructure);

    /* 使能 ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* ADC 复位校准 */
    ADC_ResetCalibration(ADC1);

    while (ADC_GetResetCalibrationStatus(ADC1))
    {
    }

    /* ADC 自校准 */
    ADC_StartCalibration(ADC1);

    while (ADC_GetCalibrationStatus(ADC1))
    {
    }

    /* 清除可能残留的转换标志 */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
}

/*
 * 读取 ADC1 指定通道。
 * channel 取值范围：0~3。
 */
uint16_t LightSensor_Read(uint8_t channel)
{
    uint16_t result;

    if (channel >= LIGHT_ADC_COUNT)
    {
        return 0;
    }

    ADC_RegularChannelConfig(
        ADC1,
        channel,
        1,
        LIGHT_ADC_SAMPLE_TIME);

    /* 清除上一次转换标志 */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    /* 启动一次软件转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 等待转换完成 */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
    }

    result = ADC_GetConversionValue(ADC1);

    return result;
}

/* 一次读取四路光敏传感器 */
void LightSensor_ReadAll(uint16_t value[4])
{
    value[0] = LightSensor_Read(0); /* L1：右上 */
    value[1] = LightSensor_Read(1); /* L2：右下 */
    value[2] = LightSensor_Read(2); /* L3：左下 */
    value[3] = LightSensor_Read(3); /* L4：左上 */
}

/* =========================================================
 * 追光模块内部状态
 * ========================================================= */

static int32_t lt_dx_base = 0;
static int32_t lt_dy_base = 0;
//环境光本底误差，用来消除四路光敏电阻对同一亮度的读数差异的影响
static int32_t lt_EX = 0;
static int32_t lt_EY = 0;
//未经过滤波时的即时读数差值 =dx - lt_dx_base
static int32_t lt_FX = 0;
static int32_t lt_FY = 0;
//完成一阶整数低通滤波后的读数差值
static uint16_t lt_dark_threshold = 0;
//弱光阈值， = 校准平均ADC - 400 处于弱光阈值时不会追光
static float lt_yaw_angle = LT_SERVO_CENTER;
static float lt_pitch_angle = LT_SERVO_CENTER;
//舵机当前角度
static uint8_t lt_initialized = 0;
static uint8_t lt_calibrated = 0;
//初始化标志位，没有进行初始化会自动进行初始化
static LightTrack_Status lt_status;
//校准成功标志位，未校准成功时，强制处于弱光状态，不追光

/* =========================================================
 * 追光内部辅助函数
 * ========================================================= */

static void LT_WaitMs(uint32_t ms)
{
    uint32_t start = GetTick();

    while ((uint32_t)(GetTick() - start) < ms)
    {
    }
}
//内部辅助用的一个阻塞式延迟

static float LT_AngleLimit(float angle)
{
    if (angle < LT_SERVO_MIN_ANGLE)
    {
        angle = LT_SERVO_MIN_ANGLE;
    }

    if (angle > LT_SERVO_MAX_ANGLE)
    {
        angle = LT_SERVO_MAX_ANGLE;
    }

    return angle;
}
//这里是给舵机设置钳位，使之不会超出0/180°


/*
 * 传感器布局：
 *
 * L4（左上）       L1（右上）
 * L3（左下）       L2（右下）
 *
 * ADC值越大，表示越暗。
 */
static void LT_CalculateError(
    const uint16_t light[4],
    int32_t *dx,
    int32_t *dy)
{
    //左侧减右侧读值，＞0说明左边暗
    *dx = (int32_t)light[3] + (int32_t)light[2]
        - (int32_t)light[0] - (int32_t)light[1];

	  //下侧减上侧，＞0说明下面暗
    *dy = (int32_t)light[2] + (int32_t)light[1]
        - (int32_t)light[3] - (int32_t)light[0];
}

static uint32_t LT_GetAverageADC(const uint16_t light[4])
{
    uint32_t sum;

    sum = (uint32_t)light[0]
        + (uint32_t)light[1]
        + (uint32_t)light[2]
        + (uint32_t)light[3];

    return sum / 4U;
}

static void LT_UpdateStatus(
    const uint16_t light[4],
    uint32_t avg_adc,
    uint8_t weak_light)
{
    uint8_t i;

    for (i = 0; i < LIGHT_ADC_COUNT; i++)
    {
        lt_status.light[i] = light[i];
    }

    lt_status.avg_adc = avg_adc;
    lt_status.dark_threshold = lt_dark_threshold;

    lt_status.EX = lt_EX;
    lt_status.EY = lt_EY;
    lt_status.FX = lt_FX;
    lt_status.FY = lt_FY;

    lt_status.weak_light = weak_light;

    lt_status.yaw_angle =
        (uint8_t)(lt_yaw_angle + 0.5f);

    lt_status.pitch_angle =
        (uint8_t)(lt_pitch_angle + 0.5f);
}

/* =========================================================
 * 追光模块对外接口
 * ========================================================= */

/*
 * 初始化追光控制。
 *
 * 调用前必须先初始化 TIM3 和 LightSensor。
 * 本函数初始化舵机并将两轴回中。
 */
void LightTrack_Init(void)
{
    lt_yaw_angle = LT_SERVO_CENTER;
    lt_pitch_angle = LT_SERVO_CENTER;

    Servo_SetAngle1(90);
    Servo_SetAngle2(90);

    lt_dx_base = 0;
    lt_dy_base = 0;

    lt_EX = 0;
    lt_EY = 0;
    lt_FX = 0;
    lt_FY = 0;

    lt_dark_threshold = 0;

    lt_initialized = 1;
    lt_calibrated = 0;

    lt_status.avg_adc = 0;
    lt_status.dark_threshold = 0;
    lt_status.weak_light = 1;
    lt_status.yaw_angle = 90;
    lt_status.pitch_angle = 90;
}

/*
 * 校准步骤：
 * 1. 舵机先回到90°。
 * 2. 等待500ms。
 * 3. 连续采集100组数据，每组间隔20ms。
 * 4. 计算 dx_base、dy_base。
 * 5. 计算四路ADC总平均值。
 * 6. 弱光阈值 = 校准平均值 - 400。
 */
void LightTrack_Calibrate(void)
{
    uint32_t i;
    uint64_t adc_sum = 0;
    int64_t dx_sum = 0;
    int64_t dy_sum = 0;

    uint32_t calibration_avg;

    uint16_t light[4];

    int32_t dx;
    int32_t dy;

    if (!lt_initialized)
    {
        LightTrack_Init();
    }
		//检测如果没有进行初始化就先进行初始化

    /* 校准开始前，先将两个舵机恢复到90° */
    lt_yaw_angle = LT_SERVO_CENTER;
    lt_pitch_angle = LT_SERVO_CENTER;

    Servo_SetAngle1(90);
    Servo_SetAngle2(90);

    /* 等待舵机机械运动稳定 */
    LT_WaitMs(500);

    for (i = 0; i < LT_CAL_SAMPLE_COUNT; i++)
    {
        LightSensor_ReadAll(light);

        LT_CalculateError(light, &dx, &dy);

        dx_sum += dx;
        dy_sum += dy;

        adc_sum += (uint64_t)light[0];
        adc_sum += (uint64_t)light[1];
        adc_sum += (uint64_t)light[2];
        adc_sum += (uint64_t)light[3];

        LT_WaitMs(LT_CAL_SAMPLE_INTERVAL);
    }

  //记录环境光本底误差值
    lt_dx_base =
        (int32_t)(dx_sum / LT_CAL_SAMPLE_COUNT);

    lt_dy_base =
        (int32_t)(dy_sum / LT_CAL_SAMPLE_COUNT);

    /* 四路、全部采样的总平均值 */
    calibration_avg =
        (uint32_t)(adc_sum /
        ((uint64_t)LT_CAL_SAMPLE_COUNT * 4U));

    /*
     * 防止 uint16_t 无符号减法下溢。
     * 若校准均值不大于400，则阈值设为0。
     */
    if (calibration_avg > LT_DARK_MARGIN)
    {
        lt_dark_threshold =
            (uint16_t)(calibration_avg - LT_DARK_MARGIN);
    }
    else
    {
        lt_dark_threshold = 0;
    }

    /* 清除校准前的误差和滤波历史 */
    lt_EX = 0;
    lt_EY = 0;
    lt_FX = 0;
    lt_FY = 0;

    lt_calibrated = 1;

    lt_status.avg_adc = calibration_avg;
    lt_status.dark_threshold = lt_dark_threshold;
    lt_status.weak_light = 0;
    lt_status.yaw_angle = 90;
    lt_status.pitch_angle = 90;
}

/*
 * 更新一次追光控制。
 * 建议每20ms调用一次。
 */
void LightTrack_Update(void)
{
    uint16_t light[4];

    uint32_t avg_adc;
    uint8_t weak_light;

    int32_t dx;
    int32_t dy;

    float step;

    if (!lt_initialized)
    {
        LightTrack_Init();
    }

    LightSensor_ReadAll(light);

    LT_CalculateError(light, &dx, &dy);

    lt_EX = dx - lt_dx_base;
    lt_EY = dy - lt_dy_base;
		//减去本底误差得到一个相对准确的差值
		

    /* 一阶整数滤波 */
    lt_FX += (lt_EX - lt_FX) / LT_FILTER_DIVISOR;
    lt_FY += (lt_EY - lt_FY) / LT_FILTER_DIVISOR;

    avg_adc = LT_GetAverageADC(light);

    /*
     * 校准后，只有ADC平均值低于阈值才表示
     * 光照比校准环境明显更强。
     */
    if (!lt_calibrated || lt_dark_threshold == 0U)
    {
        weak_light = 1U;
    }
		// 未完成校准时，保持弱光保护。
    else
    {
        weak_light =
            (avg_adc >= lt_dark_threshold) ? 1U : 0U;
    }
		//看看ADC读数的平均值是不是在弱光阈值之上

    if (!weak_light)
    {
        step = LT_SERVO_SPEED
             * ((float)LT_SAMPLE_INTERVAL / 1000.0f);
			//舵机每次转动的步长，由更新频率和最大角速度决定

        /* Yaw：水平追光 */
        if (lt_FX > LT_X_DEADBAND)
        {
            lt_yaw_angle += step;
        }
        else if (lt_FX < -LT_X_DEADBAND)
        {
            lt_yaw_angle -= step;
        }

        /* Pitch：竖直追光 */
        if (lt_FY > LT_Y_DEADBAND)
        {
            lt_pitch_angle -= step;
        }
        else if (lt_FY < -LT_Y_DEADBAND)
        {
            lt_pitch_angle += step;
        }
				//正负都存在了死区，那边暗往那边走

        lt_yaw_angle = LT_AngleLimit(lt_yaw_angle);
        lt_pitch_angle = LT_AngleLimit(lt_pitch_angle);

        Servo_SetAngle1((uint8_t)(lt_yaw_angle + 0.5f));
        Servo_SetAngle2((uint8_t)(lt_pitch_angle + 0.5f));
				//+0.5f是四舍五入取整·
    }

    LT_UpdateStatus(light, avg_adc, weak_light);
}

/* 返回最近一次更新后的状态 */
const LightTrack_Status *LightTrack_GetStatus(void)
{
    return &lt_status;
}
