
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* TIM4_CH1 -> PB6, TIM4_CH2 -> PB7; pulse width in microseconds */
void Servo_Init(void);
void Servo_SetPulse1(uint16_t pulse_us);
void Servo_SetPulse2(uint16_t pulse_us);
void Servo_SetAngle1(uint8_t angle); /* YAW: PB6 */
void Servo_SetAngle2(uint8_t angle); /* PITCH: PB7 */

#endif
