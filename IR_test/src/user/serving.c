#include "serving.h"
#include "buzzer_song.h"
#include <stdbool.h>
#include "angle_variance.h"
static u8 valve_state = 0;
static volatile SERVING_HIT_STATE serving_hit_state = SERVING_NULL;
static SERVING_CALI_STATE cali_state = SERVING_CALI_NULL;
static u8 serving_current_set = 0;
static u16 shuttle_drop_delay_ms[SERVING_SET_COUNT] = {SERVING_SHUTTLE_DROP_DELAY_DEFAULT};
static s16 serving_hit_speed[SERVING_SET_COUNT] = {SERVING_HIT_SPEED_DEFAULT};
static u32 cali_start_full_ticks = 0;
static u32 cali_lock_full_ticks = 0;

static s32 serving_encoder_target = 0;
static s32 prev_switch_pressed_encoder_val = 0;
static s32 cali_encoder_target = 0;
static u32 serving_start_hitting_full_ticks = 0;
static u32 serving_stop_hitting_full_ticks = 0;

static u32 serving_start_full_ticks = 0;

static bool serving_calibrated = false;

static void valve_set(u8 i)
{
	valve_state = i;
	//#warning VALVE GPIO FLIPPED
	gpio_write(SERVING_VALVE_GPIO, (BitAction) valve_state);	
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = SERVING_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	motor_set_acceleration(SERVING_MOTOR, SERVING_MOTOR_ACC);
	
	serving_start_hitting_full_ticks = 0;
	serving_stop_hitting_full_ticks = 0;
	
	cali_start_full_ticks = 0;
	
	for (u8 i = 0; i < SERVING_SET_COUNT; ++i) {
		shuttle_drop_delay_ms[i] = SERVING_SHUTTLE_DROP_DELAY_DEFAULT;
		serving_hit_speed[i] = SERVING_HIT_SPEED_DEFAULT;
	}	
	
}

/**
	* @brief Calibrate the serving part
	* @retval True if the calibrate can starts, as no serving or calibration is ongoing
	*/
bool serving_cali_start(void)
{
	if (cali_state == SERVING_CALI_NULL && serving_hit_state == SERVING_NULL) {
		cali_state = SERVING_CALI_START;
		serving_update();
		return true;
	} else {
		buzzer_control_note(5, 80, NOTE_E, 5);
		return false;
	}
}

/**
	* @brief Start serving
	* @retval True if the serving can starts, as no serving or calibration is ongoing
	*/
bool serving_hit_start(void)
{
	if (cali_state == SERVING_CALI_NULL && serving_hit_state == SERVING_NULL) {
		serving_hit_state = SERVING_PRE_DELAY;
		serving_calibrated = false;
		serving_start_full_ticks = get_full_ticks();
		serving_update();
		return true;
	} else {
		buzzer_control_note(3, 100, NOTE_E, 5);
		return false;
	}

}


void serving_cali_update(void)
{
	// Check timeout 
	if (cali_state != SERVING_CALI_NULL && cali_start_full_ticks > 0) {
		if (cali_start_full_ticks + SERVING_CALI_TIMEOUT <= get_full_ticks()) {
			motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);
			serving_calibrated = false;
			cali_state = SERVING_CALI_NULL;
			cali_start_full_ticks = 0;
			PLAY_FAIL_MUSIC2;
		}
	}
	
	switch (cali_state) {
		case SERVING_CALI_NULL:
			if (serving_hit_state == SERVING_NULL) {
				// Not serving currently
				if (serving_calibrated) {
					// Locking
					motor_set_vel(SERVING_MOTOR, 0, CLOSE_LOOP);
				} else {
					// Free
					motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);
				}
			}
		
		break;
		
		case SERVING_CALI_START:
			valve_set(0);
			cali_start_full_ticks = get_full_ticks();
			u32 encoder_pos_to_switch = p_mod(get_encoder_value(SERVING_MOTOR) - prev_switch_pressed_encoder_val, SERVING_MOTOR_ENCODER_CYCLE);
			if (get_serving_calibrated() || get_serving_switch()) {
				cali_state = SERVING_CALI_UNCALI;
			} else if (prev_switch_pressed_encoder_val != 0 && encoder_pos_to_switch < SERVING_MOTOR_ENCODER_CYCLE * 1 / 2) {
				cali_state = SERVING_CALI_REPRESS_SWITCH;
			} else {
				cali_state = SERVING_CALI_TO_SWITCH;
			}
			serving_cali_update();
		break;
		
		case SERVING_CALI_REPRESS_SWITCH:
			if (!get_serving_switch()) {
				motor_set_vel(SERVING_MOTOR, SERVING_REPRESS_SPEED, SERVING_REPRESS_MODE);
			} else {
				cali_state = SERVING_CALI_UNCALI;
			}	
		break;
		
		case SERVING_CALI_UNCALI:
		{
			static u16 cont_off = 0;	/* Continous off */
			
			if (get_serving_switch()) {
				motor_set_vel(SERVING_MOTOR, SERVING_UNCALI_SPEED, SERVING_UNCALI_MODE);
				cont_off = 0;
			} else {
				++cont_off;
				if (cont_off >= SERVING_UNCALI_SWITCH_OFF_COUNT) {
					cali_state = SERVING_CALI_TO_SWITCH;
				}
				
				// serving_cali_update();
			}
		}
		break;
		
		case SERVING_CALI_TO_SWITCH:
		{
			#warning sometimes wrong detection, need to be debugged
			static u16 cont_on = 0;
			
			if (!get_serving_switch()) {
				motor_set_vel(SERVING_MOTOR, SERVING_CALI_SPEED, SERVING_CALI_MODE);
				cont_on = 0;
			} else {
				++cont_on;
				
				if (cont_on >= SERVING_CALI_SWITCH_ON_COUNT) {
					prev_switch_pressed_encoder_val = get_encoder_value(SERVING_MOTOR);
					cali_encoder_target = get_encoder_value(SERVING_MOTOR) + SERVING_CALI_ENCODER_AFTER_SWITCH;
					cali_state = SERVING_CALI_SWITCH_PRESSED;
					serving_cali_update();
				}
			}
		}
		break;
		
		case SERVING_CALI_SWITCH_PRESSED:
		{
			s32 encoder_val = get_encoder_value(SERVING_MOTOR); 
			if ((SERVING_CALI_ENCODER_AFTER_SWITCH < 0 && encoder_val < cali_encoder_target)
				|| (SERVING_CALI_ENCODER_AFTER_SWITCH > 0 && encoder_val > cali_encoder_target)
			) {
				cali_encoder_target = 0;
				cali_lock_full_ticks = get_full_ticks();
				cali_state = SERVING_CALI_LOCK;
				serving_cali_update();
			} else {
				motor_set_vel(SERVING_MOTOR, SERVING_CALI_AFTER_SWITCH_SPEED, SERVING_CALI_AFTER_SWITCH_MODE);
			}
		}
		break;
		
		case SERVING_CALI_LOCK:
			motor_set_vel(SERVING_MOTOR, 0, CLOSE_LOOP);
			
			if (cali_lock_full_ticks + SERVING_CALI_LOCK_TIME_MS <= get_full_ticks()) {
				serving_calibrated = true;
				cali_state = SERVING_CALI_NULL;
				cali_start_full_ticks = 0;
				PLAY_OKAY_MUSIC1;
			}
			
		break;
		
	}
	
}
void serving_hit_update(void)
{
	switch (serving_hit_state) {
		case SERVING_NULL:
			// Not calibrating currenty
		break;
		
		case SERVING_PRE_DELAY:
			if (serving_start_full_ticks == 0 || serving_start_full_ticks + SERVING_PRE_DELAY_MAX < get_ticks() || get_angle_variance() <= SERVING_ANGLE_VARIANCE_MAX) {
				// Timeout OR angle stable, continue to start serve
				serving_start_full_ticks = 0;
				serving_hit_state = SERVING_START;
				serving_update();
				buzzer_control_note(1, 100, NOTE_G, 6);
			}
		break;
		
		case SERVING_START:
			valve_set(1);
			serving_hit_state = SERVING_SHUTTLECOCK_DROPPED;
			TIM_SetCompare1(SERVING_TIM, shuttle_drop_delay_ms[serving_current_set] * 10);		// Set CC1 CCR Value
			TIM_SetCounter(SERVING_TIM, 1);										// Reset counter
			TIM_Cmd(SERVING_TIM, ENABLE);
			TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, ENABLE);		// Interrupt for CC1
			TIM_ClearFlag(SERVING_TIM, TIM_IT_CC1);
			TIM_ClearITPendingBit(SERVING_TIM, TIM_IT_CC1);
		
			buzzer_control_note(1, 500, NOTE_G, 7);
			serving_hit_state = SERVING_SHUTTLECOCK_DROPPED;
		break;
		
		case SERVING_SHUTTLECOCK_DROPPED:
			/* Wait for the CC1 interrupt */
			
		break;
		
		case SERVING_RACKET_START_HITTING:
			/* Disable the CC1 interrupt */
			TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, DISABLE);
			TIM_Cmd(SERVING_TIM, DISABLE);
			
			serving_start_hitting_full_ticks = get_full_ticks();
			
			serving_encoder_target = get_encoder_value(SERVING_MOTOR) + SERVING_HIT_ENCODER_DIFF;
			#warning PREV
			//motor_set_vel(SERVING_MOTOR, serving_hit_speed, SERVING_HIT_MODE);
			serving_hit_state = SERVING_RACKET_HITTING;
			
			
		break;
		
		case SERVING_RACKET_HITTING:
		
		{
			bool timeout = serving_start_hitting_full_ticks + SERVING_HIT_TIMEOUT <= get_full_ticks();
			s32 encoder_val = get_encoder_value(SERVING_MOTOR); 
			if ((SERVING_HIT_ENCODER_DIFF < 0 && encoder_val < serving_encoder_target)
				|| (SERVING_HIT_ENCODER_DIFF > 0 && encoder_val > serving_encoder_target)
				|| timeout)	// Timeout
			{
				// Target fulfilled
				#warning prev
				motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);	// Motor stop (0 open loop)
				serving_hit_state = SERVING_RACKET_STOP_HITTING;
				serving_stop_hitting_full_ticks = get_full_ticks();
				serving_encoder_target = 0;
				if (!timeout) {
					PLAY_OKAY_MUSIC2;
				} else {
					PLAY_FAIL_MUSIC2;
				}
				serving_start_hitting_full_ticks = 0;
			} else {
				motor_set_vel(SERVING_MOTOR, serving_hit_speed[serving_current_set], OPEN_LOOP);
			}
		}
		break;
		
		case SERVING_RACKET_STOP_HITTING:
			if (serving_stop_hitting_full_ticks == 0 
			|| serving_stop_hitting_full_ticks + SERVING_HIT_STOP_DELAY <= get_full_ticks()
			) {
				valve_set(0);
				serving_hit_state = SERVING_NULL;
				serving_stop_hitting_full_ticks = 0;
			}
			motor_set_vel(SERVING_MOTOR, 0, OPEN_LOOP);
			
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

bool get_serving_calibrated(void)
{
	return serving_calibrated;
}

void serving_uncalibrate(void)
{
	// stop any current calibration
	cali_state = SERVING_CALI_NULL;
	cali_start_full_ticks = 0;
	cali_lock_full_ticks = 0; 
	serving_calibrated = false;
}


u8 get_serving_set(void)
{
	return serving_current_set;
}

void set_serving_set(u8 i)
{
	if (i >= SERVING_SET_COUNT || serving_hit_state != SERVING_NULL) {
		return;
	}
	serving_current_set = i;
}

u8 get_serving_switch(void)
{
	//#warning SWITCH GPIO FLIPPED
	return gpio_read_input(SERVING_SWITCH_GPIO);
}


u16 get_shuttle_drop_delay(void)
{
	return shuttle_drop_delay_ms[serving_current_set];
}

void set_shuttle_drop_delay(u16 delay_ms)
{
	if (delay_ms > 999 || delay_ms == 0) {return;}
	shuttle_drop_delay_ms[serving_current_set] = delay_ms;
}

s16 get_serving_hit_speed(void)
{
	return serving_hit_speed[serving_current_set];
}

void set_serving_hit_speed(s16 speed)
{
	if (speed < -2000 || speed > 2000 || speed == 0) {return;}
	serving_hit_speed[serving_current_set] = speed;
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
			motor_set_vel(SERVING_MOTOR, serving_hit_speed[serving_current_set], SERVING_HIT_MODE);
			serving_update();
			buzzer_control_note(2, 100, NOTE_A, 7);
			//TIM_SetCounter(SERVING_TIM, 0);										// Reset counter
			//TIM_ITConfig(SERVING_TIM, TIM_IT_CC1, ENABLE);
			
			
		}
	}
}


