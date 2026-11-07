云台工程双舵机第一步（TIM4 PWM）

本补丁新增：
- Lib/Servo.h
- Lib/Servo.c
并修改 user/main.c：TIM3_init() 后调用 Servo_Init()，将两路脉宽设为 1500 us。
template.uvprojx 已加入 Servo.c/Servo.h。

引脚：
- 舵机 1 信号：PB6 / TIM4_CH1
- 舵机 2 信号：PB7 / TIM4_CH2

PWM 参数（假设系统时钟 72 MHz，APB1 分频为 2）：
- PSC = 71，计数频率 1 MHz（1 tick = 1 us）
- ARR = 19999，周期 20000 us = 20 ms（50 Hz）
- 初始 CCR1/CCR2 = 1500 us

重要：
1. 舵机使用独立且足够供电能力的 5 V 电源；电源 GND 必须与 STM32 GND 共地。
2. 先不连接舵机负载或确保机械结构不会卡死，再上电验证。
3. 此版本未设置 TIM4 重映射，依赖 PB6/PB7 默认映射；若工程其他地方修改 AFIO_MAPR，需要复核。
4. Servo_SetPulse1/2 的接口参数单位是微秒，当前限幅 500~2500 us。舵机机械端点有差异，首次测试可先在 1000~2000 us 范围内试验。
5. 本补丁未修改 TIM3、按键、OLED、USART 及其逻辑。
