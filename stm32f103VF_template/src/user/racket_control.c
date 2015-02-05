#include "racket_control.h"

static bool command_received = false;
static bool switch_on = false;
static bool calibrated = false;
static u32 racket_last_cmd_received_time = 0;
static u32 prev_encoder_value = 0;
static u32 current_encoder_distance = 0;

void racket_init(void)
{
	register_special_char_function('u', racket_received_command);
	register_special_char_function(']', racket_calibrate);
	register_special_char_function('y', open_pneumatic);
	register_special_char_function('j', close_pneumatic);

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	// switch init
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	// pneumatic init	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
	
	/* EXTI configuration */
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* NVIC configuration */
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)){
			switch_on = true;
		} else {
			switch_on = false;
		}
		racket_last_cmd_received_time = get_full_ticks();
	  EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

void racket_calibrate(void)
{
	if (!switch_on) {
		motor_set_vel(MOTOR5, 300, OPEN_LOOP);
	}
	calibrated = true;
}

void racket_update(void)
{
	if (get_full_ticks() - racket_last_cmd_received_time > RACKET_TIMEOUT) {
		command_received = false;
	}
	if ((!switch_on || command_received) && calibrated) {
		motor_set_vel(MOTOR5, 900, OPEN_LOOP);
	} else {
		motor_set_vel(MOTOR5, 0, OPEN_LOOP);
	}
}

void racket_received_command(void)
{
	command_received = true;
	racket_last_cmd_received_time = get_full_ticks();
}

bool did_receive_command(void)
{
	return command_received;
}

void open_pneumatic(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_15);
}

void close_pneumatic(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
}
