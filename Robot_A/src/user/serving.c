#include "serving.h"
#include "buzzer_song.h"

static u8 valve_state = 0;
static SERVING_HIT_STATE serving_hit_state = SERVING_NULL;
static SERVING_CALI_STATE cali_state = SERVING_CALI_NULL;
static u16 shuttle_drop_delay_ms = SERVING_SHUTTLE_DROP_DELAY_DEFAULT;
static s16 serving_hit_speed = SERVING_HIT_SPEED_DEFAULT;

static s32 serving_encoder_target = 0;
static s32 cali_encoder_target = 0;
static u32 serving_stop_hitting_time = 0;

static u8 serving_calibrated = 0;

static void valve_set(u8 i)
{
	valve_state = i;
	#warning VALVE GPIO FLIPPED
	gpio_write(SERVING_VALVE_GPIO, (BitAction) !valve_state);	
}


void serving_init(void)
{
	// Switch
	gpio_init(SERVING_SWITCH_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IPU, 1);
	
	// Valve
	gpio_init(SERVING_VALVE_GPIO, GPIO_Speed_10MHz, GPIO_Mode_Out_PP, 1);
	
	valve_set(0);
	serving_hit_state = SERVING_NULL;
	cali_state = SERVING_CALI_NULL;
	
	// Timer init
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library
	TIM_OCInitTypeDef  TIM_OCInitStructure;   

	RCC_APB1PeriphClockCmd(SERVING_TIM_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 65535;	                				       
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 10000 - 1;	// 10000Hz    
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(SERVING_TIM, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(SERVING_TIM, ENABLE);
  //TIM_Cmd(SERVING_TIM, ENABLE);	
	
	// Make use of output compare (CCx interrupt) 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         //set "high" to be effective output
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             //produce output when counter < CCR
	//TIM_OCInitStructure.TIM_Pulse = SERVING_SHUTTLECOCK_DROPPING_TIME;                               // this part sets the initial CCR value
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    // this part enable the output
	
	TIM_OC1Init(SERVING_TIM, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(SERVING_TIM, TIM_OCPreload_Disable); 				// Enable CCR to be reset
	
	TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, DISABLE);	
	TIM_ClearFlag(SERVING_TIM, TIM_IT_CC1);
	TIM_ClearITPendingBit(SERVING_TIM, TIM_IT_CC1);												 // Clear Interrupt bits	 // Counter Enable
	
	TIM_SetCounter(SERVING_TIM, 0);
	TIM_Cmd(SERVING_TIM, DISABLE);
	
	// Serving_IRQn NVIC init
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = SERVING_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	motor_set_acceleration(SERVING_MOTOR, SERVING_MOTOR_ACC);
	
}

void serving_cali_start(void)
{
	if (cali_state == SERVING_CALI_NULL && serving_hit_state == SERVING_NULL) {
		cali_state = SERVING_CALI_START;
		serving_update();
	} else {
		buzzer_control_note(5, 80, NOTE_E, 5);
	}
}

void serving_hit_start(void)
{
	if (cali_state == SERVING_CALI_NULL && serving_hit_state == SERVING_NULL) {
		serving_hit_state = SERVING_START;
		serving_calibrated = 0;
		serving_update();
	} else {
		buzzer_control_note(3, 100, NOTE_E, 5);
	}

}


void serving_cali_update(void)
{
	switch (cali_state) {
		case SERVING_CALI_START:
			if (get_serving_calibrated() || get_serving_switch()) {
				cali_state = SERVING_CALI_UNCALI;
			} else {
				cali_state = SERVING_CALI_TO_SWITCH;
			}
			serving_cali_update();
		break;
		
		case SERVING_CALI_UNCALI:
			if (get_serving_switch()) {
				motor_set_vel(SERVING_MOTOR, SERVING_UNCALI_SPEED, SERVING_UNCALI_MODE);
			} else {
				cali_state = SERVING_CALI_TO_SWITCH;
				// serving_cali_update();
			}
		break;
		
		case SERVING_CALI_TO_SWITCH:
			if (!get_serving_switch()) {
				motor_set_vel(SERVING_MOTOR, SERVING_CALI_SPEED, SERVING_CALI_MODE);
			} else {
				cali_encoder_target = get_encoder_value(SERVING_MOTOR) + SERVING_CALI_ENCODER_AFTER_SWITCH;
				cali_state = SERVING_CALI_SWITCH_PRESSED;
				serving_cali_update();
			}
		break;
		
		case SERVING_CALI_SWITCH_PRESSED:
		{
			s32 encoder_val = get_encoder_value(SERVING_MOTOR); 
			if ((SERVING_CALI_ENCODER_AFTER_SWITCH < 0 && encoder_val < cali_encoder_target)
				|| (SERVING_CALI_ENCODER_AFTER_SWITCH > 0 && encoder_val > cali_encoder_target)
			) {
				cali_state = SERVING_CALI_LOCK;
				serving_cali_update();
			} else {
				motor_set_vel(SERVING_MOTOR, SERVING_CALI_AFTER_SWITCH_SPEED, SERVING_CALI_AFTER_SWITCH_MODE);
			}
		}
		break;
		
		case SERVING_CALI_LOCK:
			motor_set_vel(SERVING_MOTOR, 0, CLOSE_LOOP);
			serving_calibrated = 1;
			cali_state = SERVING_CALI_NULL;
			buzzer_play_song(MARIO_BEGIN, 80, 0);
		break;
		
	}
	
}
void serving_hit_update(void)
{
	switch (serving_hit_state) {
		case SERVING_START:
			valve_set(1);
			serving_hit_state = SERVING_SHUTTLECOCK_DROPPED;
			TIM_SetCompare1(SERVING_TIM, shuttle_drop_delay_ms * 10);		// Set CC1 CCR Value
			TIM_SetCounter(SERVING_TIM, 1);										// Reset counter
			TIM_Cmd(SERVING_TIM, ENABLE);
			TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, ENABLE);		// Interrupt for CC1
			TIM_ClearFlag(SERVING_TIM, TIM_IT_CC1);
			TIM_ClearITPendingBit(SERVING_TIM, TIM_IT_CC1);
		
			serving_hit_state = SERVING_SHUTTLECOCK_DROPPED;
		break;
		
		case SERVING_SHUTTLECOCK_DROPPED:
			
		break;
		
		case SERVING_RACKET_START_HITTING:
			TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, DISABLE);
			TIM_Cmd(SERVING_TIM, DISABLE);
			

			serving_encoder_target = get_encoder_value(SERVING_MOTOR) + SERVING_HIT_ENCODER_DIFF;
			motor_set_vel(SERVING_MOTOR, serving_hit_speed, SERVING_HIT_MODE);
			serving_hit_state = SERVING_RACKET_HITTING;
			
			
		break;
		
		case SERVING_RACKET_HITTING:
		
		{
			s32 encoder_val = get_encoder_value(SERVING_MOTOR); 
			if ((SERVING_HIT_ENCODER_DIFF < 0 && encoder_val < serving_encoder_target)
				|| (SERVING_HIT_ENCODER_DIFF > 0 && encoder_val > serving_encoder_target)
			) {
				// Target arrived
				motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);	// Motor stop (0 open loop)
				serving_hit_state = SERVING_RACKET_STOP_HITTING;
				serving_stop_hitting_time = get_full_ticks();
				buzzer_control_note(2, 200, NOTE_C, 7);
			} else {
				motor_set_vel(SERVING_MOTOR, serving_hit_speed, OPEN_LOOP);
			}
		}
		break;
		
		case SERVING_RACKET_STOP_HITTING:
			if (serving_stop_hitting_time == 0 || serving_stop_hitting_time + SERVING_HIT_STOP_DELAY <= get_full_ticks()) {
				serving_hit_state = SERVING_RECALI;
				serving_stop_hitting_time = 0;
			} else {
				motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);
			}
			
		break;
		
		case SERVING_RECALI:
			//motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);
			buzzer_play_song(MARIO_END, 80, 0); 
			valve_set(0);
			serving_hit_state = SERVING_NULL;
			serving_cali_start();
		break;
	}	
}


void serving_update(void) 
{
	serving_hit_update();
	serving_cali_update();
}


s32 get_serving_encoder(void)
{
	return get_encoder_value(SERVING_MOTOR);
}

s32 get_serving_hit_encoder_target(void)
{
	return serving_encoder_target;
}

s32 get_serving_cali_encoder_target(void)
{
	return cali_encoder_target;
}

u8 get_serving_calibrated(void)
{
	return serving_calibrated;
}

u8 get_serving_switch(void)
{
	//#warning SWITCH GPIO FLIPPED
	return gpio_read_input(SERVING_SWITCH_GPIO);
}


u16 get_shuttle_drop_delay(void)
{
	return shuttle_drop_delay_ms;
}

void set_shuttle_drop_delay(u16 delay_ms)
{
	shuttle_drop_delay_ms = delay_ms;
}

s16 get_serving_hit_speed(void)
{
	return serving_hit_speed;
}

void set_serving_hit_speed(s16 speed)
{
	serving_hit_speed = speed;
}

SERVING_CALI_STATE get_serving_cali_state(void)
{
	return cali_state; 
}

SERVING_HIT_STATE get_serving_hit_state(void)
{
	return serving_hit_state;
}

SERVING_IRQn_Handler
{
  if (TIM_GetITStatus(SERVING_TIM, TIM_IT_CC1) != RESET) {
    TIM_ClearFlag(SERVING_TIM, TIM_IT_CC1);
		TIM_ClearITPendingBit(SERVING_TIM, TIM_IT_CC1);
		
		if (serving_hit_state == SERVING_SHUTTLECOCK_DROPPED) {
			serving_hit_state = SERVING_RACKET_START_HITTING;
			#warning
			serving_update();
			buzzer_control_note(2, 100, NOTE_A, 7);
			//TIM_SetCounter(SERVING_TIM, 0);										// Reset counter
			//TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, ENABLE);
			
			
		} else if (serving_hit_state == SERVING_RACKET_HITTING) {
			
		}
	}
}

