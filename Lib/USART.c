
#include "USART.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* 串口接收环形缓冲区：在本文件中定义一次 */
volatile uint8_t rx_buffer[RX_BUF_SIZE];
volatile uint16_t rx_read = 0;
volatile uint16_t rx_write = 0;
volatile uint8_t rx_overflow = 0;

void USART_INIT(void)
{
    GPIO_InitTypeDef GI;
    USART_InitTypeDef UI;
    NVIC_InitTypeDef NI;

    USART_StructInit(&UI);

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_USART1 |
        RCC_APB2Periph_GPIOB,
        ENABLE
    );

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* USART1 TX：PA9 */
    GI.GPIO_Pin = GPIO_Pin_9;
    GI.GPIO_Mode = GPIO_Mode_AF_PP;
    GI.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GI);

    /* USART3 RX：PB11 */
    GI.GPIO_Pin = GPIO_Pin_11;
    GI.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GI);

    /* USART1：115200，8N1，仅发送 */
    UI.USART_BaudRate = 115200;
    UI.USART_Parity = USART_Parity_No;
    UI.USART_StopBits = USART_StopBits_1;
    UI.USART_Mode = USART_Mode_Tx;
    UI.USART_WordLength = USART_WordLength_8b;
    UI.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &UI);

    /* USART3：115200，8N1，仅接收 */
    UI.USART_Mode = USART_Mode_Rx;
    USART_Init(USART3, &UI);

    /* USART3 接收中断 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NI.NVIC_IRQChannel = USART3_IRQn;
    NI.NVIC_IRQChannelPreemptionPriority = 0;
    NI.NVIC_IRQChannelSubPriority = 0;
		//让串口接收数据的中断的优先级高于TIM3的1ms时基中断，防止丢数据
    NI.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NI);

    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);
        uint16_t next = (rx_write + 1) % RX_BUF_SIZE;

        if (next != rx_read)
        {
            rx_buffer[rx_write] = data;
            rx_write = next;
        }
        else
        {
            rx_overflow = 1;
        }
    }
}

void USART1_SendByte(uint8_t data)
{
    USART_SendData(USART1, data);

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void USART1_SendString(char *str)
{
    while (*str != '\0')
    {
        USART1_SendByte(*str);
        str++;
    }
}

void USART1_Printf(const char *Format, ...){
	char format_buffer[256];
	va_list argptr;
	
	__va_start(argptr, Format);
	
	vsprintf(format_buffer, Format, argptr);
	
	__va_end(argptr);
	
	USART1_SendString(format_buffer);
}
