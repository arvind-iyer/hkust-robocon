#include "motion.h"

u8 last_motor1 = 100;
u8 last_motor2 = 100;

void motion_set_motor(s32 compare,u8 direction)
{
	if (compare>1799)
		compare=1799;
	else if (compare<0)
		compare=0;

	TIM_SetCompare3(TIM3,compare);
	
	if(direction==0)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_4);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	}
	else
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOC,GPIO_Pin_4);
	}
	

}





