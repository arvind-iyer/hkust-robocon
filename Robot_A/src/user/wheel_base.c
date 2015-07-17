#include "wheel_base.h"

static WHEEL_BASE_VEL wheel_base_vel_target = {0, 0, 0},
		wheel_base_vel_real = {0, 0, 0};
static const CLOSE_LOOP_FLAG wheel_base_close_loop_flag = CLOSE_LOOP;
static u8 wheel_base_speed_mode = WHEEL_BASE_DEFAULT_SPEED_MODE;
static u32 wheel_base_bluetooth_vel_last_update = 0;
static char wheel_base_bluetooth_last_char = 0;
static u32 wheel_base_last_can_tx = 0;


static u8 wheel_base_pid_flag = 0;
static POSITION target_pos = {0, 0, 0};
static PID wheel_base_pid = {0, 0, 0};


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
					#warning
					//wheel_base_set_vel(x_vel * speed_ratio / 100, y_vel * speed_ratio / 100, w_vel * speed_ratio / 100);
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

/**
  * @brief Handler for wheel base character decoding
  */
static void wheel_base_char_bluetooth_decode(u8 id, u8 length, u8* data)
{
  switch (id) {
    case BLUETOOTH_WHEEL_BASE_CHAR_ID:
      if (length == 1) {
        wheel_base_bluetooth_last_char = data[0];
      }
    break;
  }
}


/**
	* @brief Initialization of wheel base, including bluetooth rx filter, and all related variables
	*/
void wheel_base_init(void)
{
	bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_VEL_ID, 0xF0, wheel_base_bluetooth_decode);
  bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_AUTO_POS_ID, 0xF0, wheel_base_auto_bluetooth_decode);
  bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_CHAR_ID, 0xFF, wheel_base_char_bluetooth_decode);
	wheel_base_vel_target.x = wheel_base_vel_target.y = wheel_base_vel_target.w = 0;
	wheel_base_bluetooth_vel_last_update = 0;
	wheel_base_last_can_tx = 0;
	wheel_base_tx_acc();
}

/**
	* @brief Set speed mode
	* @param The speed mode
	* @retval None
	*/
void wheel_base_set_speed_mode(u8 s)
{
	if (s < WHEEL_BASE_SPEED_MODE_COUNT) {
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
	motor_set_acceleration(MOTOR_BOTTOM_RIGHT, WHEEL_BASE_BR_ACC);
	motor_set_acceleration(MOTOR_BOTTOM_LEFT, WHEEL_BASE_BL_ACC);
	motor_set_acceleration(MOTOR_TOP_LEFT, WHEEL_BASE_TL_ACC);
	motor_set_acceleration(MOTOR_TOP_RIGHT, WHEEL_BASE_TR_ACC);
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
	wheel_base_vel_target.x = x;
	wheel_base_vel_target.y = y;
	wheel_base_vel_target.w = w;
}

/**
	* @brief Get current wheel base velocity
	* @param None.
	* @retval Current wheel base velocity.
	*/
WHEEL_BASE_VEL wheel_base_get_vel_target(void)
{
	return wheel_base_vel_target;
}

WHEEL_BASE_VEL wheel_base_get_vel_real(void)
{
	return wheel_base_vel_real;
}

char wheel_base_bluetooth_get_last_char(void) 
{
   return wheel_base_bluetooth_last_char;
}

WHEEL_BASE_VEL wheel_base_vel_sub(WHEEL_BASE_VEL a, WHEEL_BASE_VEL b)
{
	WHEEL_BASE_VEL result = {a.x - b.x, a.y - b.y, a.w - b.w};
	return result;
}

s32 wheel_base_vel_scalar(WHEEL_BASE_VEL v) 
{
	return Sqrt(v.x * v.x + v.y * v.y);
} 



/**
	* @brief Update the wheel base speed through CAN transmission. (TO BE CALLED REGULARLY)
	* @param None.
	* @retval None.
	*/
void wheel_base_update(void)
{
	wheel_base_stop();
	return;
  /** 
    * TODO1: Use wheel_base_set_vel(x,y,w) to control the FOUR wheel base motor
    * TODO2: If there is not any Bluetooth RX data after BLUETOOTH_WHEEL_BASE_TIMEOUT, stop the motors
  
    */
		
		s32 speed[4];
		
		// Acceleration
		WHEEL_BASE_VEL diff = wheel_base_vel_sub(wheel_base_vel_target, wheel_base_vel_real);
		s32 diff_scale = (s32) wheel_base_vel_scalar(diff);
		
		// XY
		if (diff_scale > 0) {
			if (diff_scale < WHEEL_BASE_XY_ACC) {
				wheel_base_vel_real.x = wheel_base_vel_target.x;
				wheel_base_vel_real.y = wheel_base_vel_target.y;
			} else {
				wheel_base_vel_real.x += (diff.x * WHEEL_BASE_XY_ACC) / diff_scale;
				wheel_base_vel_real.y += (diff.y * WHEEL_BASE_XY_ACC) / diff_scale;
			}
		}
		// W
		if (diff.w != 0) {
			if (Abs(diff.w) < WHEEL_BASE_W_ACC) {
				wheel_base_vel_real.w = wheel_base_vel_target.w;
			} else {
				if (diff.w > 0) {
					wheel_base_vel_real.w += WHEEL_BASE_W_ACC;
				} else if (diff.w < 0) {
					wheel_base_vel_real.w -= WHEEL_BASE_W_ACC;
				}
			}
		}
		
		
		speed[0] = WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel_real.x + wheel_base_vel_real.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel_real.w / 1000;
		speed[1] = WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel_real.x - wheel_base_vel_real.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel_real.w / 1000;
		speed[2] = WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel_real.x - wheel_base_vel_real.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel_real.w / 1000;
		speed[3] = WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel_real.x + wheel_base_vel_real.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel_real.w / 1000;
		
		for (u8 i = 0; i < 4; ++i) {
			speed[i] /= WHEEL_BASE_SPEED_SCALE_DOWN;
		}
		
		motor_set_vel(MOTOR_BOTTOM_RIGHT,	speed[0], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_BOTTOM_LEFT,	speed[1], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_TOP_LEFT,			speed[2], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_TOP_RIGHT,		speed[3], wheel_base_close_loop_flag);

		//wheel_base_vel_real.x = wheel_base_vel.x;
		//wheel_base_vel_real.y = wheel_base_vel.y;
		//wheel_base_vel_real.w = wheel_base_vel.w;		
	
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
void wheel_base_set_pid(PID pid)
{
	wheel_base_pid.Kp = pid.Kp;
	wheel_base_pid.Ki = pid.Ki;
	wheel_base_pid.Kd = pid.Kd;
}

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


/**
	* @brief Stop the wheel base by stopping all motors to be open loop speed ZERO
	*/
	
void wheel_base_stop(void)
{
		motor_set_vel(MOTOR_BOTTOM_RIGHT,	0, OPEN_LOOP);
		motor_set_vel(MOTOR_BOTTOM_LEFT,	0, OPEN_LOOP);
		motor_set_vel(MOTOR_TOP_LEFT,			0, OPEN_LOOP);
		motor_set_vel(MOTOR_TOP_RIGHT,		0, OPEN_LOOP);	
}

