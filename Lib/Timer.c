
#include "Timer.h"

static volatile uint32_t CurrentTime = 0;


void TIM3_init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitTypeDef TI;
	
	TI.TIM_CounterMode = TIM_CounterMode_Up ;//计算器（CNT）计数方式设为上计数
	TI.TIM_Prescaler = 71; //预分频器（PSC）设为71
	TI.TIM_Period  = 999; //自动重装寄存器（ARR）设为999
	TI.TIM_RepetitionCounter = 0;//重复计数器（RCR）设为0
	TI.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM3,&TI);	
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);//使能tim3中断
			
	NVIC_InitTypeDef  NI;//初始化NVIC模块
	
	NI.NVIC_IRQChannel = TIM3_IRQn;
	NI.NVIC_IRQChannelPreemptionPriority = 1;
	NI.NVIC_IRQChannelSubPriority  = 0;
	NI.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NI);
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	
	TIM_Cmd(TIM3,ENABLE);
}
uint32_t GetTick(void){			//获取当前时间
	return CurrentTime;
}

void Delay(uint32_t t){			//阻塞式延迟函数
	uint32_t endtime = CurrentTime + t;
	while ( CurrentTime < endtime );
}

void Timer_Tick(void)
{
    CurrentTime++;
}

