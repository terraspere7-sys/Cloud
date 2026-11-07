#include "stm32f10x.h"
#include <string.h>
#include "math.h"

#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "USART.h"
#include "Command.h"
#include "SI2C.h"
#include "OLED.h"
#include "Servo.h"
#include "SI2C_MPU.h"
#include "MPU_6050.h"
#include "TEST.h"
#include "LightSensor.h"


//----------------变量定义------------------//

volatile uint8_t button_event = 0;      //回传按键事件
uint8_t mode = 1;		//用来记录当前模式
uint8_t light_track_calibrated = 0;

uint32_t mpu_tick = 0;
uint32_t mpu_retry_tick = 0;
uint32_t light_track_tick = 0;
uint32_t led2_tick = 0;	
uint32_t led1_tick = 0;		
//用来记录led上点亮或者熄灭的时间

uint8_t oled_ok = 0;//表示OLED当前状态
uint32_t oled_check_tick = 0;//OLED上次自检的时间点
uint32_t oled_retry_tick = 0;//OLED上次尝试重新初始化的时间点

uint8_t mpu_ok = 0;//表示mpu6050当前状态

//OLED的显示状态由主循环和命令模块共同维护
char oled_key_status[6] = "NONE";
char oled_cmd_status[8] = "NONE";

int main(void)
{
	
	Key_init();
	
	LED_Init();
	
	USART_INIT();
			
	TIM3_init();
	
	//双舵机的初始化
	Servo_Init();
	
	Servo_SetAngle1(90); 
	Servo_SetAngle2(90); 
	//初始化完成后自动调整至中位
	
	LightSensor_Init();
	LightTrack_Init();
	
	MY_SI2C_INIT();
	
	MY_SI2C_MPU_INIT();
	
	MPU_INIT();
	
	oled_ok = OLED_Init();
			
	if (oled_ok){
    /* OLED 正常：LED1 常亮 */
    LED_On(LED1);

    OLED_ShowString(0, 20, "CLOUDHOUSE");

    OLED_ShowString(2, 0, "MODE");
    OLED_ShowNum(2, 36, mode);

    OLED_ShowString(4, 0, "KEY");
    OLED_ShowString(4, 36, oled_key_status);

    OLED_ShowString(6, 0, "CMD");
    OLED_ShowString(6, 36, oled_cmd_status);
		OLED_ShowAxisTargets();
	}
	else{
    /* OLED 故障：通过过串口报告 */
    USART1_SendString("ERR OLED INIT\r\n");   
	}
	
	MPU_CalibrateStart();
	
	USART1_SendString("please keep mpu still after led self check");
	
	self_check(); 
	//流水灯上电自检，采用的是阻塞式的计时，正好这段时间供使用者调整好姿态板放置	
	
	
	
	while(1)
	{
		MPU_CalibrateTask(200);
		
		if (MPU_CalibrateResult() == MPU_CAL_FAILED)
    {
			mpu_ok = 0;
    }
		else if(MPU_CalibrateResult() == MPU_CAL_SUCCESS){
			mpu_ok = 1;
		}
						
		Cmd_Process();
		
		//运行时每5s检查一遍OLED状态		
	if ((uint32_t)(GetTick() - oled_check_tick) >= 5000){
    oled_check_tick = GetTick();

    if (oled_ok)
    {
			//检查OLED连接状态以及上一次写入操作ACK是否错误
        if (!OLED_CheckConnection() || OLED_HasError())
        {
            oled_ok = 0;
            oled_retry_tick = GetTick();
            USART1_SendString("ERR OLED OFFLINE\r\n");
        }
    }
	}


	if (!oled_ok && (uint32_t)(GetTick() - oled_retry_tick) >= 5000){
    oled_retry_tick = GetTick();

    if (OLED_Init())
    {
        oled_ok = 1;
        USART1_SendString("OLED RECOVERED\r\n");

				/* 恢复连接后重绘完整页面 */
        OLED_ShowString(0, 20, "CLOUDHOUSE");
        OLED_ShowString(2, 0, "MODE");
        OLED_ShowNum(2, 36, mode);
        OLED_ShowString(4, 0, "KEY");
        OLED_ShowString(4, 36, oled_key_status);
        OLED_ShowString(6, 0, "CMD");
        OLED_ShowString(6, 36, oled_cmd_status);
        OLED_ShowAxisTargets();
    }
    else
    {
        USART1_SendString("ERR OLED RETRY\r\n");
    }
}

//LED1充当OLED的状态指示灯时代码
if (oled_ok && mpu_ok)
{
    LED_On(LED1);
}
else {
	if ((uint32_t)(GetTick() - led1_tick) >= 500)
	{
    led1_tick = GetTick();
    LED_Toggle(LED1);
	}
}

		if ((uint32_t)(GetTick() - mpu_tick) > 5000u){
			mpu_tick = GetTick();
			
			if (mpu_ok){
				if(!MPU_ConnectCheck()){
					mpu_ok = 0;
					USART1_SendString("MPU CONNECT LOST\r\n");
					mpu_retry_tick = GetTick();
				}
			}
		}
		
		if ((uint32_t)(GetTick() - mpu_retry_tick) > 5000u && !mpu_ok){
			mpu_retry_tick =GetTick();
			if (MPU_ConnectCheck()){
				mpu_ok = 1;
				USART1_SendString("MPU CONNECT RECOVER\r\n");
			}
		}
		
//根据按键状态来做出响应，1：短按，2：长按
		switch (button_event){
			case 1:
				USART1_SendString("KEY S\r\n");
				button_event = 0;
			//这里的{}是用来划定一个局部作用域给old_mode让这个变量只在作用域内生效
			{
				uint8_t old_mode = mode;
			//局域变量记录上一个模式是什么
        mode++;			
        if (mode > 3) mode = 1;
			
			//如果上一个模式不是1且当前切换到的模式是1，重置舵机
        if (mode == 1 && old_mode != 1)
        {
          Servo_SetAngle1(90);
          Servo_SetAngle2(90);
          yaw_target_angle = 90;
          pitch_target_angle = 90;
          OLED_ShowAxisTargets();
        }
			}
			
			if (mode == 3)
{
    light_track_calibrated = 0;
}
			
				//按键事件发生意味着模式发生切换，回传当前模式并更改OLED内容
  			USART1_SendString("MODE ");
        USART1_SendByte('0' + mode);
        USART1_SendString("\r\n");	
				
				OLED_ClearLine(2);
				OLED_ShowString(2, 0, "MODE");
				OLED_ShowNum(2, 36, mode);
			
				OLED_ClearLine(4);
				OLED_ShowString(4, 0, "KEY");
				OLED_ShowString(4, 36, "SHORT");
				break;
			case 2:
				USART1_SendString("KEY L\r\n");
				button_event = 0;
			//因为上面的是old_mode是在局部作用域内的，所以这里再次定义是可以的
                {
                    uint8_t old_mode = mode;
                    if (mode <= 1) mode = 3;
                    else mode--;
                    if (mode == 1 && old_mode != 1)
                    {
                        Servo_SetAngle1(90);
                        Servo_SetAngle2(90);
                        yaw_target_angle = 90;
                        pitch_target_angle = 90;
                        OLED_ShowAxisTargets();
                    }
                }

								
				if (mode == 3)
{
    light_track_calibrated = 0;
}


				USART1_SendString("MODE ");
        USART1_SendByte('0' + mode);
        USART1_SendString("\r\n");
				
				OLED_ClearLine(2);
				OLED_ShowString(2, 0, "MODE");
				OLED_ShowNum(2, 36, mode);
				
				OLED_ClearLine(4);
				OLED_ShowString(4, 0, "KEY");
				OLED_ShowString(4, 36, "LONG");
				break;
			default:
				break;
	}						
			
		switch(mode){
			case 1:
				LED_Off(LED2);
				break;
			case 2:
				if(mpu_ok != 1){
					break ;
				}
				LED_On(LED2);
				Data_Process();
				Gimbal_ControlTask();
				break;
			case 3:				
				if (!light_track_calibrated)
    {
        LightTrack_Calibrate();
        light_track_calibrated = 1;

        USART1_SendString("LIGHT CAL OK\r\n");
    }

    /* 运行光敏追踪任务 */
    /* 每隔20ms更新一次追光控制 */
if ((uint32_t)(GetTick() - light_track_tick) >= 20)
{
    light_track_tick = GetTick();
    LightTrack_Update();
}

    /* 保留原有 LED2 每500ms闪烁功能 */
    if ((uint32_t)(GetTick() - led2_tick) >= 500)
    {
        led2_tick = GetTick();
        LED_Toggle(LED2);
    }

    break;
}
		}
	}


void TIM3_IRQHandler(void){			//TIM3的中断函数，负责1ms时基以及按键状态的扫描
	if (TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET){
		
		Timer_Tick();
		Key_Scan();
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}		
}

