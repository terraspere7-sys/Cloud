#include "Key.h"
#include "Timer.h"

Key_t a = {Key_IDLE, 0, 0};
Key_t *p = &a;

void Key_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
	
	//以下为初始化按钮的引脚
	GPIO_InitStructure.GPIO_Pin = Button;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;

	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void Key_Scan(void){
		uint8_t level = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8);//读取按钮状态，0为按下，1为松开
		
		switch (p->State ){
			case Key_IDLE:
				if (level == 0){   //检测到按下，进入消抖阶段
					p->State = Key_DeBounce;
					p->pressed_tick = GetTick ();
				}
				break;
			case Key_DeBounce:
				if(GetTick() - p->pressed_tick > 20){  //做一个20ms的消抖
					if (level == 0){
						p->State = Key_Pressed;
					}
					else{
						p->pressed_tick  = 0;
						p->State = Key_IDLE;
					}
				}
				break;
			case Key_Pressed:
				if(level == 1){   //检测到松开进入松开消抖
					p->released_tick = GetTick();
					p->State = Key_Released;
				}
				break;
			case Key_Released:
				if(GetTick() - p->released_tick > 20){  //做一个20ms的消抖
					if(level == 1){
						uint32_t TimeLength = p->released_tick - p->pressed_tick;//计算按下时长
						
						if (TimeLength < 50){   //小于50ms视为抖动
							p->pressed_tick = 0;
							p->released_tick = 0;
							p->State = Key_IDLE ;
						}
						else if(TimeLength >= 50 && TimeLength < 1000){  //50ms到1秒内视为短按
							button_event = 1;
						}
						else if(TimeLength >= 1000){   //1秒以上视为长按
							button_event = 2;
						}
						p->State = Key_IDLE ;
					}
					else{
						p->State = Key_Pressed;//判定为抖动，返回按下状态
					}
				}
				break;
	}
}
