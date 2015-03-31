#include "wheel_base.h"
#include "wheel_base_pid.h"
#include "special_char_handler.h"
#include "xbc_mb.h"
#include "xbc_button.h"

static WHEEL_BASE_VEL wheel_base_vel = {0, 0, 0},
		wheel_base_vel_prev = {-1, -1, -1};
static const CLOSE_LOOP_FLAG wheel_base_close_loop_flag = CLOSE_LOOP;
static u8 wheel_base_speed_mode = WHEEL_BASE_DEFAULT_SPEED_MODE;
static u32 wheel_base_bluetooth_vel_last_update = 0;

static const u8 WHEEL_BASE_PID_MANUAL_SPEED = 10;
		
static int wheel_base_acc = 300;

// static u32 wheel_base_last_can_tx = 0;
		
static u8 wheel_base_pid_flag = 0;
static POSITION target_pos = {0, 0, 0};
//static PID wheel_base_pid = {0, 0, 0};

/**
	* @brief Handler for the bluetooth RX with id 0x4?
	* @param id: ID of the RX package
	* @param length: Data length
	* @param data: The array with the data length
	* @retval None
	*/
static void wheel_base_bluetooth_decode(u8 id, u8 length, u8* data)
{
	switch (id) {
		case BLUETOOTH_WHEEL_BASE_VEL_ID:
			if (length == 3) {
				s8	x_vel = (s8)data[0],
						y_vel = (s8)data[1],
						w_vel = (s8)data[2];
				
				s8 data_range[2] = {-100, 100};
				if (x_vel >= data_range[0] && x_vel <= data_range[1] &&
					y_vel >= data_range[0] && y_vel <= data_range[1] &&
					w_vel >= data_range[0] && w_vel <= data_range[1]) {
          
          u16 speed_ratio = SPEED_MODES[wheel_base_speed_mode]; 
					if (wheel_base_get_pid_flag() == 1) {
						speed_ratio = WHEEL_BASE_PID_MANUAL_SPEED;
					}
					wheel_base_set_vel(x_vel * speed_ratio / 100, y_vel * speed_ratio / 100, w_vel * speed_ratio / 100);
					wheel_base_bluetooth_vel_last_update = get_full_ticks();
				}
			}
		break;
			
		case BLUETOOTH_WHEEL_BASE_SPEED_MODE_ID:
			if (length == 1) {
				if (data[0] <= 9) {
					wheel_base_set_speed_mode(data[0]);
				}
			}
		break;
	}
}

static void wheel_base_auto_bluetooth_decode(u8 id, u8 length, u8* data)
{
	switch (id) {
		case BLUETOOTH_WHEEL_BASE_AUTO_POS_ID:
			if (length == 6) {
				target_pos.x = ((data[0] << 8) & 0xFF00) | (data[1] & 0xFF);
				target_pos.y = ((data[2] << 8) & 0xFF00) | (data[3] & 0xFF);
				target_pos.angle = (((data[4] << 8) & 0xFF00) | (data[5] & 0xFF)) * 10;
				
			}
		break;
		
		case BLUETOOTH_WHEEL_BASE_AUTO_START_ID:
			if (length == 0) {
				wheel_base_pid_on();
			}
		break;

			
		case BLUETOOTH_WHEEL_BASE_AUTO_STOP_ID:
			wheel_base_pid_off();
		break;
	}
}

static void stop_all_motors(void)
{
	while(1) {
		motor_lock(MOTOR1);
		motor_lock(MOTOR2);
		motor_lock(MOTOR3);
		motor_lock(MOTOR4);
		motor_lock(MOTOR5);
		motor_lock(MOTOR6);
	}
}

/**
	* @brief Initialization of wheel base, including bluetooth rx filter, and all related variables
	*/
void wheel_base_init(void)
{
	bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_VEL_ID, 0xF0, wheel_base_bluetooth_decode);
  bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_AUTO_POS_ID, 0xF0, wheel_base_auto_bluetooth_decode);
	register_special_char_function(' ', stop_all_motors);
	wheel_base_vel.x = wheel_base_vel.y = wheel_base_vel.w = 0;
	wheel_base_bluetooth_vel_last_update = 0;
//	wheel_base_last_can_tx = 0;
	wheel_base_tx_acc();
}

/**
	* @brief Set speed mode
	* @param The speed mode
	* @retval None
	*/
void wheel_base_set_speed_mode(u8 s)
{
	if (s <= 9) {
		wheel_base_speed_mode = s;
	}
}

/** 
	* @brief Get speed mode
	* @param None
	* @retval Get wheel base speed mode.
	*/
u8 wheel_base_get_speed_mode(void)
{
	return wheel_base_speed_mode;
}

/**
	* @brief Set acceleration
	* @param a: acceleration parameter (default 50)
	* @retval None.
	*/
void wheel_base_tx_acc(void)
{
	motor_set_acceleration(MOTOR_BOTTOM_RIGHT,WHEEL_BASE_BR_ACC);
	motor_set_acceleration(MOTOR_BOTTOM_LEFT,WHEEL_BASE_BL_ACC);
	motor_set_acceleration(MOTOR_TOP_LEFT,WHEEL_BASE_TL_ACC);
	motor_set_acceleration(MOTOR_TOP_RIGHT,WHEEL_BASE_TR_ACC);
}


/**
	* @brief Set wheel base velocity (NO CAN TRANSMISSION IN THIS FUNCTION!)
	* @param x:	Velocity in x direction (right as +ve).
	* @param y:	Velocity in y direction (up as +ve).
	* @param w:	Velocity in omega (clockwise as +ve).
	* @retval Velocity type value that has been set.
	*/
void wheel_base_set_vel(s32 x, s32 y, s32 w)
{
	wheel_base_vel.x = x * 100;
	wheel_base_vel.y = y * 100;
	wheel_base_vel.w = w * 100;
}

/**
	* @brief Get target wheel base velocity
	* @param None.
	* @retval target wheel base velocity.
	*/
WHEEL_BASE_VEL wheel_base_get_vel(void)
{
	return wheel_base_vel;
}

// get current speed

WHEEL_BASE_VEL wheel_base_get_prev_vel(void)
{
	return wheel_base_vel_prev;
}

// get when the last manual command was received

u32 wheel_base_get_last_manual_timer(void)
{
	return wheel_base_bluetooth_vel_last_update;
}

/**
	* @brief Check if the wheel base velocity is different from the previous one
	* @param None
	* @retval True if there is any different
	*/
u8 wheel_base_vel_diff(void)
{
	return wheel_base_vel.x != wheel_base_vel_prev.x || wheel_base_vel.y != wheel_base_vel_prev.y || wheel_base_vel.w != wheel_base_vel_prev.w;
}

// updates wheel base velocity using fake acceleration

static s32 wheel_base_vel_update(s32 current_vel, s32 target_vel, s32 wheel_base_accel)
{
	// base case
	if (target_vel <= current_vel + wheel_base_accel && target_vel >= current_vel - wheel_base_accel)
	{
		return target_vel;
	}
	else if (target_vel > current_vel + wheel_base_accel)
	{
		return current_vel + wheel_base_accel;
	}
	else 
	{
		return current_vel - wheel_base_accel;
	}
}

// function to set speed using xbox buttons directly

void wheel_base_set_xbox_vel(void)
{
	int x_vel = xbc_get_joy(XBC_JOY_LX) / 10;
	int y_vel = xbc_get_joy(XBC_JOY_LY) / 10;
	int w_vel = (xbc_get_joy(XBC_JOY_RT) - xbc_get_joy(XBC_JOY_LT)) * 100 / 255;
	u16 speed_ratio = SPEED_MODES[wheel_base_speed_mode];
	wheel_base_set_vel(x_vel * speed_ratio / 100, y_vel * speed_ratio / 100, w_vel * speed_ratio / 100);
}

/**
	* @brief Update the wheel base speed through CAN transmission. (TO BE CALLED REGULARLY)
	* @param None.
	* @retval None.
	*/
void wheel_base_update(void)
{
  /** 
    * TODO1: Use wheel_base_set_vel(x,y,w) to control the FOUR wheel base motor
    * TODO2: If there is not any Bluetooth RX data after BLUETOOTH_WHEEL_BASE_TIMEOUT, stop the motors
  
    */
	
	// detect when timeout is reached to stop motors if no pid
	if ((get_full_ticks() - wheel_base_bluetooth_vel_last_update) > BLUETOOTH_WHEEL_BASE_TIMEOUT && (wheel_base_get_pid_flag() != 1 || (wheel_base_get_pid_flag() && get_pid_stat()))) {
		wheel_base_set_vel(0, 0, 0);
	}
	
	// detect when timeout is reached to stop motors for xbox when PID is off
	if (xbc_get_connection() != XBC_DISCONNECTED && (wheel_base_get_pid_flag() != 1 ||
		(wheel_base_get_pid_flag() == 1 && (get_full_ticks() - xbc_get_received_nonzero_speed_timer()) < BLUETOOTH_WHEEL_BASE_TIMEOUT)))
	{
		// xbox is connected, set target velocity based on it
		wheel_base_set_xbox_vel();
	}
	
	// fake acceleration
	if (wheel_base_vel_diff()){
		int x_error = abs(wheel_base_vel.x);
		int y_error = abs(wheel_base_vel.y);
		if (x_error == 0)
		{
			x_error = 1;
		}
		if (y_error == 0)
		{
			y_error = 1;
		}
		s32 x_wheel_base_acc, y_wheel_base_acc;
		if (x_error > y_error) {
			x_wheel_base_acc = wheel_base_acc;
			y_wheel_base_acc = wheel_base_acc * y_error / x_error;
		} else {
			x_wheel_base_acc = wheel_base_acc * x_error / y_error;
			y_wheel_base_acc = wheel_base_acc;
		}
		wheel_base_vel_prev.x = wheel_base_vel_update(wheel_base_vel_prev.x, wheel_base_vel.x, x_wheel_base_acc);
		wheel_base_vel_prev.y = wheel_base_vel_update(wheel_base_vel_prev.y, wheel_base_vel.y, y_wheel_base_acc);
		wheel_base_vel_prev.w = wheel_base_vel_update(wheel_base_vel_prev.w, wheel_base_vel.w, wheel_base_acc);
	}
	
	motor_set_vel(MOTOR_TOP_LEFT, ((-wheel_base_vel_prev.y - wheel_base_vel_prev.x) * WHEEL_BASE_XY_VEL_RATIO + wheel_base_vel_prev.w * WHEEL_BASE_W_VEL_RATIO) / 100000, wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_TOP_RIGHT, ((wheel_base_vel_prev.y - wheel_base_vel_prev.x) * WHEEL_BASE_XY_VEL_RATIO + wheel_base_vel_prev.w * WHEEL_BASE_W_VEL_RATIO) / 100000, wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_BOTTOM_LEFT, ((-wheel_base_vel_prev.y  + wheel_base_vel_prev.x) * WHEEL_BASE_XY_VEL_RATIO	+ wheel_base_vel_prev.w * WHEEL_BASE_W_VEL_RATIO) / 100000, wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_BOTTOM_RIGHT, ((wheel_base_vel_prev.y + wheel_base_vel_prev.x) * WHEEL_BASE_XY_VEL_RATIO + wheel_base_vel_prev.w * WHEEL_BASE_W_VEL_RATIO) / 100000, wheel_base_close_loop_flag);
}

/**
	* @brief Send the wheel base position (x, y, w) through Bluetooth
	* @param None.
	* @retval None.
	*/
void wheel_base_tx_position(void)
{
	s16 x = get_pos()->x,
			y = get_pos()->y,
			w = (get_pos()->angle / 10);
	
	u8 data[6];
	
	// POS XY
	data[0] = (x >> 8) & 0xFF;
	data[1] = x & 0xFF;
	data[2] = (y >> 8) & 0xFF;
	data[3] = y & 0xFF;
	data[4] = (w >> 8) & 0xFF;
	data[5] = w & 0xFF;	
	
	bluetooth_tx_package(BLUETOOTH_WHEEL_BASE_POS_ID, 6, data);
}

/**
	* @brief Set PID
	* @param PID to be set
	* @retval None.
	*/

/*
void wheel_base_set_pid(PID pid)
{
	wheel_base_pid.Kp = pid.Kp;
	wheel_base_pid.Ki = pid.Ki;
	wheel_base_pid.Kd = pid.Kd;
}
*/

POSITION wheel_base_get_target_pos(void)
{
	return target_pos;
}

void wheel_base_set_target_pos(POSITION pos)
{
	target_pos = pos;
}

void wheel_base_pid_on(void)
{
	wheel_base_pid_flag = 1;
}

void wheel_base_pid_off(void)
{
	wheel_base_pid_flag = 0;
}

u8 wheel_base_get_pid_flag(void)
{
	return wheel_base_pid_flag;
}

void wheel_base_override_set_vel(s32 x, s32 y, s32 w)
{
	u16 speed_ratio = SPEED_MODES[wheel_base_speed_mode]; 
	wheel_base_set_vel(x * speed_ratio / 100, y * speed_ratio / 100, w * speed_ratio / 100);
	wheel_base_bluetooth_vel_last_update = get_full_ticks();
}

void wheel_base_override_change_speed(void)
{
	wheel_base_set_speed_mode((wheel_base_get_speed_mode() + 1) % 10);
}
