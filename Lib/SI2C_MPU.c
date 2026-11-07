#include "USART.h"
#include "SI2C_MPU.h"

void MY_SI2C_MPU_INIT(void)
{
    GPIO_InitTypeDef GI;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GI.GPIO_Mode = GPIO_Mode_Out_OD;
    GI.GPIO_Pin = SI2C_SCL_mpu | SI2C_SDA_mpu;
    GI.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GI);

    /* 开漏输出释放总线；需要正确的外部上拉 */
    GPIO_SetBits(GPIOA, SI2C_SCL_mpu | SI2C_SDA_mpu);
}

void scl_write_mpu(uint8_t level)
{
    GPIO_WriteBit(GPIOA, SI2C_SCL_mpu, level ? Bit_SET : Bit_RESET);
}

void sda_write_mpu(uint8_t level)
{
    GPIO_WriteBit(GPIOA, SI2C_SDA_mpu, level ? Bit_SET : Bit_RESET);
}

uint8_t sda_read_mpu(void)
{
    return GPIO_ReadInputDataBit(GPIOA, SI2C_SDA_mpu) == Bit_SET ? 1U : 0U;
}

void delay_us_mpu(uint32_t t)
{
    volatile uint32_t n = t * 8U;
    while (n-- > 0U)
    {
        /* 软件延时 */
    }
}

void sendstart_mpu(void)
{
    sda_write_mpu(1);
    scl_write_mpu(1);
    delay_us_mpu(2);

    sda_write_mpu(0);
    delay_us_mpu(2);

    scl_write_mpu(0);
    delay_us_mpu(2);
}

void sendstop_mpu(void)
{
    scl_write_mpu(0);
    sda_write_mpu(0);
    delay_us_mpu(1);

    scl_write_mpu(1);
    delay_us_mpu(1);

    sda_write_mpu(1);
    delay_us_mpu(1);
}

/* 返回值：0=收到 ACK，1=收到 NACK */
uint8_t sendbyte_mpu(uint8_t byte)
{
    uint8_t ack;
    int8_t i;

    for (i = 7; i >= 0; i--)
    {
        scl_write_mpu(0);
        sda_write_mpu((byte & (1U << i)) ? 1U : 0U);
        delay_us_mpu(2);

        scl_write_mpu(1);
        delay_us_mpu(2);
    }

    /* 第九个时钟：释放 SDA，由从机决定 ACK/NACK */
    scl_write_mpu(0);
    sda_write_mpu(1);
    delay_us_mpu(2);

    scl_write_mpu(1);
    delay_us_mpu(2);
    ack = sda_read_mpu();

    scl_write_mpu(0);
    delay_us_mpu(2);

    return ack;
}

uint8_t receivebyte_mpu(uint8_t ack)
{
    uint8_t byte = 0;
    int8_t i;

    for (i = 7; i >= 0; i--)
    {
        scl_write_mpu(0);
        sda_write_mpu(1);
        delay_us_mpu(2);

        scl_write_mpu(1);
        delay_us_mpu(2);

        if (sda_read_mpu())
        {
            byte |= (uint8_t)(1U << i);
        }
    }

    /* ack 非零发送 ACK；ack 为零发送 NACK（保持原接口约定） */
    scl_write_mpu(0);
    sda_write_mpu(ack ? 0U : 1U);
    delay_us_mpu(2);

    scl_write_mpu(1);
    delay_us_mpu(2);

    scl_write_mpu(0);
    sda_write_mpu(1);
    delay_us_mpu(2);

    return byte;
}
