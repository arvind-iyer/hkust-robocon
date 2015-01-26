#include "wheel_base.h"

static WHEEL_BASE_VEL wheel_base_vel = {0, 0, 0},
		wheel_base_vel_prev = {-1, -1, -1};
static const CLOSE_LOOP_FLAG wheel_base_close_loop_flag = CLOSE_LOOP;
static u8 wheel_base_speed_mode = WHEEL_BASE_DEFAULT_SPEED_MODE;
static u32 wheel_base_bluetooth_last_update = 0;
static u32 wheel_base_last_can_tx = 0;



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
		case BLUETOOTH_WHEEL_BASE_MANUAL_ID:
			if (length == 3) {
				s8	x_vel = (s8)data[0],
						y_vel = (s8)data[1],
						w_vel = (s8)data[2];
				
				s8 data_range[2] = {-100, 100};
				if (x_vel >= data_range[0] && x_vel <= data_range[1] &&
					y_vel >= data_range[0] && y_vel <= data_range[1] &&
					w_vel >= data_range[0] && w_vel <= data_range[1]) {
					wheel_base_set_vel(x_vel, y_vel, w_vel);
					wheel_base_bluetooth_last_update = get_full_ticks();
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

/**
	* @brief Initialization of wheel base, including bluetooth rx filter, and all related variables
	*/
void wheel_base_init(void)
{
	bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_MANUAL_ID, 0xF0, wheel_base_bluetooth_decode);
	wheel_base_vel.x = wheel_base_vel.y = wheel_base_vel.w = 0;
	wheel_base_bluetooth_last_update = 0;
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
	wheel_base_vel.x = x;
	wheel_base_vel.y = y;
	wheel_base_vel.w = w;
}

/**
	* @brief Get current wheel base velocity
	* @param None.
	* @retval Current wheel base velocity.
	*/
WHEEL_BASE_VEL wheel_base_get_vel(void)
{
	return wheel_base_vel;
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

/**
	* @brief Update the wheel base speed through CAN transmission. (TO BE CALLED REGULARLY)
	* @param None.
	* @retval None.
	*/
void wheel_base_update(void)
{
	if (get_full_ticks() - wheel_base_bluetooth_last_update > BLUETOOTH_WHEEL_BASE_TIMEOUT) {
		wheel_base_vel.x = wheel_base_vel.y = wheel_base_vel.w = 0;
	}
	
	if (wheel_base_vel_diff() || get_full_ticks() - wheel_base_last_can_tx > WHEEL_BASE_REGULAR_CAN_TX)
	{
		u16 speed_ratio = SPEED_MODES[wheel_base_speed_mode];
		u8 i = 0;
		s32 speed[4];
		
		speed[0] = WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel.x + wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000;
		speed[1] = WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel.x - wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000;
		speed[2] = WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel.x - wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000;
		speed[3] = WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel.x + wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000;
		
		// Multiplying the speed ratio
		for (i = 0; i < 4; ++i) {
			speed[i] = speed_ratio * speed[i] / 100;
		}
		
		motor_set_vel(MOTOR_BOTTOM_RIGHT,	speed[0], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_BOTTOM_LEFT,	speed[1], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_TOP_LEFT,			speed[2], wheel_base_close_loop_flag);
		motor_set_vel(MOTOR_TOP_RIGHT,		speed[3], wheel_base_close_loop_flag);

		wheel_base_vel_prev.x = wheel_base_vel.x;
		wheel_base_vel_prev.y = wheel_base_vel.y;
		wheel_base_vel_prev.w = wheel_base_vel.w;
		
		wheel_base_last_can_tx = get_full_ticks();
	}
	
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
			w = get_pos()->angle;
	
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

