#include "Timer.h"
#include "LED.h"

/* 初始化 LED 引脚 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA, ENABLE
    );

    GPIO_InitStructure.GPIO_Pin = LED1 | LED2 | LED3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 默认熄灭 */
    GPIO_ResetBits(GPIOA, LED1 | LED2 | LED3);
}

/* 点亮 LED */
void LED_On(uint16_t pin)
{
    GPIO_SetBits(GPIOA, pin);
}

/* 熄灭 LED */
void LED_Off(uint16_t pin)
{
    GPIO_ResetBits(GPIOA, pin);
}

/* 翻转 LED 状态 */
void LED_Toggle(uint16_t pin)
{
    GPIO_WriteBit(
        GPIOA,
        pin,
        GPIO_ReadOutputDataBit(GPIOA, pin) == Bit_SET
            ? Bit_RESET
            : Bit_SET
    );
}

void self_check(void){
	for (uint8_t i = 0;i < 3;i++){
		LED_On(LED1);
		Delay(200);
		LED_Off(LED1);
		LED_On(LED2);
		Delay(200);
		LED_Off(LED2);
		LED_On(LED3);
		Delay(200);
		LED_Off(LED3);
	}
	LED_On(LED1);
}
