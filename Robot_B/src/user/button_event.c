#include "button_event.h"

static u8  side_control = SIDE_LEFT;

static bool underarm_reverse = false;

// Omega PID relevant.
static int accumulated_omega = 0;
static int target_angle = 0;

static bool force_terminate = false;

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

bool is_near_the_net(void) {
	WHEEL_BASE_VEL velocity = wheel_base_get_vel();
	s32 velocity_y = velocity.y;
	return (get_sensor() < 700);
}

void gamepad_led_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);		// Red off
	GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);	// Green off
}

void gamepad_wheel_base() {
	//Analog Movement
	//Set x and y vel according to analog stick input
	int raw_vx, raw_vy, raw_temp;
	if (button_pressed(BUTTON_PS4_N) || button_pressed(BUTTON_PS4_S) ||
		  button_pressed(BUTTON_PS4_W) || button_pressed(BUTTON_PS4_E) ||
			button_pressed(BUTTON_PS4_NW) || button_pressed(BUTTON_PS4_NE) ||
			button_pressed(BUTTON_PS4_SW) || button_pressed(BUTTON_PS4_SE)
	) {
		raw_vx = 0;
		raw_vy = 0;
		if (button_pressed(BUTTON_PS4_N))
			raw_vy = 255;
		else if (button_pressed(BUTTON_PS4_S))
			raw_vy = -255;
		else if (button_pressed(BUTTON_PS4_W))
			raw_vx = -255;
		else if (button_pressed(BUTTON_PS4_E))
			raw_vx = 255;
		else if (button_pressed(BUTTON_PS4_NW)) {
			raw_vy = 180;
			raw_vx = -180;
		}
		else if (button_pressed(BUTTON_PS4_NE)) {
			raw_vy = 180;
			raw_vx = 180;
		}
		else if (button_pressed(BUTTON_PS4_SW)) {
			raw_vy = -180;
			raw_vx = -180;
		}
		else if (button_pressed(BUTTON_PS4_SE)) {
			raw_vy = -180;
			raw_vx = 180;
		}
		if (side_control == SIDE_LEFT) {
			raw_temp = raw_vx;
			raw_vx = raw_vy;
			raw_vy = -raw_temp;
		} else if (side_control == SIDE_RIGHT) {
			raw_temp = raw_vx;
			raw_vx = -raw_vy;
			raw_vy = raw_temp;
		}
	} else {
		if (side_control == SIDE_LEFT) {
			raw_vx = xbc_get_joy(XBC_JOY_LY);
			raw_vy = -xbc_get_joy(XBC_JOY_LX);
		} else if (side_control == SIDE_RIGHT) {
			raw_vx = -xbc_get_joy(XBC_JOY_LY);
			raw_vy = xbc_get_joy(XBC_JOY_LX);
		} else {
			raw_vx = xbc_get_joy(XBC_JOY_LX);
			raw_vy = xbc_get_joy(XBC_JOY_LY);
		}
	}
	
	int h = Sqrt(Sqr(raw_vx)+ Sqr(raw_vy));
	
	// Scalar Speed limit
	if (h > XBC_JOY_SCALE) {
		raw_vx = raw_vx * XBC_JOY_SCALE / h;
		raw_vy = raw_vy * XBC_JOY_SCALE / h;
	}
	
	// Set output x and y.
	s32 vx = raw_vx;
	s32 vy = raw_vy;
	
	int omega = (xbc_get_joy(XBC_JOY_RT)-xbc_get_joy(XBC_JOY_LT)) / 5;
	const int speed_factor = 10;
	
	accumulated_omega += omega % speed_factor;
	int incremental = accumulated_omega / speed_factor;
	if (incremental != 0) {
		target_angle += incremental;
		accumulated_omega -= incremental * speed_factor;
	}
	
	// changing target angle with int value.
	target_angle += omega / speed_factor;
	if (target_angle < 0) {
		target_angle += 3600;
	} else if (target_angle > 3599) {
		target_angle -= 3600;
	}
	
	wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y, target_angle});
	
	// Get angular speed based on target_angle
	int vw = pid_maintain_angle();
	
	if (!force_terminate) {
		if (is_near_the_net()) {
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);		// Red on
		} else {
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);		// Red off
		}
		
		if (get_sensor() < 700 && vy > 0) {
			vy = 0;
		}
		wheel_base_set_vel(vx,vy,vw);
	}
}

void button_event_update(void)
{
	gamepad_wheel_base();
	
	// SELECT function keys
	if (button_pressed(BUTTON_PS4_SELECT)>=1) {
		// SELECT + L1 + R1
		if (button_pressed(BUTTON_PS4_L1)>=1 && button_pressed(BUTTON_PS4_R1)>=1) {
			side_control = SIDE_NORMAL;
			buzzer_play_song(SIDE_CONTROL_NORMAL_SOUND, 120, 0);
		}
		// SELECT + L1
		else if (button_pressed(BUTTON_PS4_L1)==1) {
			side_control = SIDE_LEFT;
			buzzer_play_song(SIDE_CONTROL_LEFT_SOUND, 120, 0);
		}
		// SELECT + R1
		else if (button_pressed(BUTTON_PS4_R1)==1) {
			side_control = SIDE_RIGHT;
			buzzer_play_song(SIDE_CONTROL_RIGHT_SOUND, 120, 0);
		}
	}
	// L1 function keys
	else if (button_pressed(BUTTON_PS4_L1) >= 1) {
		// L1 + X
		if (button_pressed(BUTTON_PS4_CROSS) == 1)
			underarm_reverse = !underarm_reverse;
	}
	// Single button (except force_terminate())
	else {
		// CROSS
		if (button_pressed(BUTTON_PS4_CROSS) >= 1) {
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
		// SQUARE
		if (button_pressed(BUTTON_PS4_SQUARE) >= 1) {
			forehand_daa_la();
		} else {
			forehand_lok_la();
		}
		// TRIANGLE
		
		// CIRCLE
	}
}

u8 button_event_get_side_control(void) {
	return side_control;
}

bool is_force_terminate(void) {
	return force_terminate;
}
