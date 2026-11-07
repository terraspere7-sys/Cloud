#include "Servo.h"


void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 19999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 1500;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
		
		Servo_SetAngle1(90); 
		Servo_SetAngle2(90); 
	//初始化完成后自动调整至中位
}

void Servo_SetPulse1(uint16_t pulse_us)
{
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    TIM_SetCompare1(TIM4, pulse_us);
}

void Servo_SetPulse2(uint16_t pulse_us)
{
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    TIM_SetCompare2(TIM4, pulse_us);
}


void Servo_SetAngle1(uint8_t angle)
{
    uint16_t pulse_us;
    if (angle > 180) angle = 180;
	pulse_us = 500U + ((uint16_t)angle * 2000U) / 180U;//通过公式把角度转为频率
    Servo_SetPulse1(pulse_us);
}

void Servo_SetAngle2(uint8_t angle)
{
    uint16_t pulse_us;
    if (angle > 180) angle = 180;
    pulse_us = 500U + ((uint16_t)angle * 2000U) / 180U;
    Servo_SetPulse2(pulse_us);
}
