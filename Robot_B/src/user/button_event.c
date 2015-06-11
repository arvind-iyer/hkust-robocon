#include "button_event.h"

static u8   side_control = SIDE_RIGHT;

static u32 home_holding_count = 0;
static u32 start_and_select_holding_count = 0;
static u32 select_and_l1_holding_count = 0;
static u32 l1_and_cross_holding_count = 0;
static u32 r1_and_cross_holding_count = 0;
static u32 triangle_holding_count = 0;
static bool speed_button_released_before = false;
static bool underarm_reverse = false;
static u8 chetaudaaidang = 0;

static u16 speed_ratio;
static s32 axis_speed;
static s32 diagonal_speed;
static s32 angle_speed;
static s32 x_speed;
static s32 y_speed;
static s32 temp_s32;
static u32 l_analog_magnitude;

// This "trigger" means LT (L2 for PS4) & RT (R2 for PS4)
s32 button_event_trigger_value_conversion(s16 trigger_value) {
    if (trigger_value == 255 || trigger_value == -255) {
		return trigger_value;
	} else if (trigger_value > 0) {
		return (trigger_value * trigger_value + 147) / 294 + (trigger_value * 3 + 12) / 23;
	} else {
		return (- trigger_value * trigger_value - 147) / 294 + (trigger_value * 3 - 12) / 23;
	}
}

void button_event_wheel_base_set_vel(s32 x, s32 y, s32 w) {
	wheel_base_set_vel(x, y, w);
	wheel_base_vel_last_update_refresh();
}

void gamepad_led_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);		// Red off
	GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET);		// 車頭燈
	GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);	// Green off
}

void button_event_update(void)
{
	// Button holding count START
	if (button_pressed(BUTTON_XBC_XBOX)) {
		home_holding_count += 1;
	} else {
		home_holding_count = 0;
	}
	if (button_pressed(BUTTON_PS4_SELECT) && button_pressed(BUTTON_PS4_START)) {
		start_and_select_holding_count += 1;
	} else {
		start_and_select_holding_count = 0;
	}
	
	if (button_pressed(BUTTON_PS4_SELECT) && button_pressed(BUTTON_PS4_L1)) {
		select_and_l1_holding_count += 1;
	} else {
		select_and_l1_holding_count = 0;
	}
	
	if (button_pressed(BUTTON_PS4_L1) && button_pressed(BUTTON_PS4_CROSS)) {
		l1_and_cross_holding_count += 1;
	} else {
		l1_and_cross_holding_count = 0;
	}
		
	if (button_pressed(BUTTON_PS4_R1) && button_pressed(BUTTON_PS4_CROSS)) {
		r1_and_cross_holding_count += 1;
	} else {
		r1_and_cross_holding_count = 0;
	}
	
	if (button_pressed(BUTTON_PS4_TRIANGLE)) {
		triangle_holding_count += 1;
	} else {
		triangle_holding_count = 0;
	}
	// Button holding count END

	//Analog Movement
	//Set x and y vel according to analog stick input
	int raw_vx = xbc_get_joy(XBC_JOY_LX);
	int raw_vy = xbc_get_joy(XBC_JOY_LY);
	
	int h = Sqrt(Sqr(raw_vx)+ Sqr(raw_vy));
	
	// Scalar Speed limit
	if (h > XBC_JOY_SCALE) {
		raw_vx = raw_vx*XBC_JOY_SCALE / h;
		raw_vy = raw_vy*XBC_JOY_SCALE / h;
	}
	
	// Set output x and y.
	s32 vx = raw_vx;
	s32 vy = raw_vy;
	
	// Original Code
	if (triangle_holding_count == 1) {
		chetaudaaidang = (chetaudaaidang + 1) % 3;
	}
	if (chetaudaaidang == 1 || (chetaudaaidang == 2 && (get_full_ticks() / 100) % 2 == 1)) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_SET);
	} else {
		GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET);
	}
	
	// Button event START
	if (button_pressed(BUTTON_PS4_L1)) {
		// GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
	} else {
		// GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
	}
	
	if (button_pressed(BUTTON_PS4_R1)) {
		// GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_SET);
	} else {
		// GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);
	}
	
	if (button_pressed(BUTTON_PS4_L3)) {
		buzzer_play_song(BIRTHDAY_SONG, 135, 0);
	}
	if (button_pressed(BUTTON_PS4_R3)) {
		buzzer_play_song(NEW_SUPER_MARIO_BROS, 150, 0);
	}
	// Button event END
	
	// Speed mode adjustment
	if ( speed_button_released_before && (button_pressed(BUTTON_XBC_START) || button_pressed(BUTTON_XBC_BACK) ) ) {
		u8 speed_mode = wheel_base_get_speed_mode();
		if ( button_pressed(BUTTON_XBC_START) && speed_mode < 9 ) {
			speed_mode += 1;
		} else if ( button_pressed(BUTTON_XBC_BACK) && speed_mode > 0 ) {
			speed_mode -= 1;
		}
		wheel_base_set_speed_mode(speed_mode);
		switch(speed_mode) {
			case 0:
				buzzer_play_song(SPEED_0, 200, 0);
				break;
			case 1:
				buzzer_play_song(SPEED_1, 200, 0);
				break;
			case 2:
				buzzer_play_song(SPEED_2, 200, 0);
				break;
			case 3:
				buzzer_play_song(SPEED_3, 200, 0);
				break;
			case 4:
				buzzer_play_song(SPEED_4, 200, 0);
				break;
			case 5:
				buzzer_play_song(SPEED_5, 200, 0);
				break;
			case 6:
				buzzer_play_song(SPEED_6, 200, 0);
				break;
			case 7:
				buzzer_play_song(SPEED_7, 200, 0);
				break;
			case 8:
				buzzer_play_song(SPEED_8, 200, 0);
				break;
			case 9:
				buzzer_play_song(SPEED_9, 200, 0);
				break;
			default:
				FAIL_MUSIC;
		}
		speed_button_released_before = false;
	} else if( !button_pressed(BUTTON_XBC_START) && !button_pressed(BUTTON_XBC_BACK) ) {
		speed_button_released_before = true;
	}
	
	
	// Holding event START
	if (home_holding_count == 1 || start_and_select_holding_count == 1 ) {
		if (side_control == SIDE_NORMAL) {
			side_control = SIDE_RIGHT;
			buzzer_play_song(SIDE_CONTROL_ON_SOUND, 120, 0);
		} else {
			side_control = SIDE_NORMAL;
			buzzer_play_song(SIDE_CONTROL_OFF_SOUND, 120, 0);
		}
	}
	
	//sensor_off();
	//sensor_on();
	if (select_and_l1_holding_count == 1) {
		side_control = SIDE_LEFT;
		buzzer_play_song(SIDE_CONTROL_LEFT_SOUND, 120, 0);
	}
	
	if ((button_pressed(BUTTON_XBC_Y) && button_pressed(BUTTON_XBC_B)) == false) {
		if (button_pressed(BUTTON_XBC_Y)) {
			sensor_decrease_delay();
		} else if (button_pressed(BUTTON_XBC_B)) {
			sensor_increase_delay();
		}
	}
	
	// Rackets & sensors
	
	if (l1_and_cross_holding_count == 1) {
		underarm_reverse = !underarm_reverse;
	}
	
	if (r1_and_cross_holding_count >= 1) {
		sensor_on();
	} else {
		sensor_off();		
		if (button_pressed(BUTTON_XBC_A)) {
			if (underarm_reverse == false)
				underarm_daa_la();
			else
				underarm_lok_la();
		} else {
			if (underarm_reverse == false)
				underarm_lok_la();
			else
				underarm_daa_la();
		}
	} 
	
	if (button_pressed(BUTTON_XBC_X)) {
		forehand_daa_la();
	} else {
		forehand_lok_la();
	}
	
}

u8 button_event_get_side_control(void) {
	return side_control;
}

u32 button_event_get_l_analog_magnitude(void) {
	return l_analog_magnitude;
}
