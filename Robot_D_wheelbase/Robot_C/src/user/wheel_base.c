#include "wheel_base.h"

static WHEEL_BASE_VEL wheel_base_vel = {0, 0, 0},
		wheel_base_vel_prev = {-1, -1, -1},
		wheel_base_target = {0, 0, 0};

static const CLOSE_LOOP_FLAG wheel_base_close_loop_flag = CLOSE_LOOP;
static u8 wheel_base_speed_mode = WHEEL_BASE_DEFAULT_SPEED_MODE;
static u32 wheel_base_bluetooth_vel_last_update = 0;
static char wheel_base_bluetooth_last_char = 0;
static u32 wheel_base_bluetooth_char_last_update = 0;
static u32 wheel_base_last_can_tx = 0;
//static u32 wheel_base_joystick_vel_last_update=1;
static u8 wheel_base_joystick_speed=30;	//0% to 100%
static u8 is_turning = 0, is_moving = 0;
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
			wheel_base_pid_off();	//abort auto_positioning
			if (length == 3) {
				s8	x_vel = (s8)data[0],
						y_vel = (s8)data[1],
						w_vel = (s8)data[2];
				
				is_turning = (w_vel != 0);
				is_moving = (x_vel!=0) || (y_vel!=0);
				
				wheel_base_set_target_pos((POSITION){get_pos()->x, get_pos()->y,get_pos()->angle});
				
				s8 data_range[2] = {-100, 100};
				if (x_vel >= data_range[0] && x_vel <= data_range[1] &&
					y_vel >= data_range[0] && y_vel <= data_range[1] &&
					w_vel >= data_range[0] && w_vel <= data_range[1]) {
          
          u16 speed_ratio = SPEED_MODES[wheel_base_speed_mode]; 
					wheel_base_set_vel(x_vel * speed_ratio / 100, y_vel * speed_ratio / 100, w_vel * speed_ratio / 100);
					wheel_base_bluetooth_vel_last_update = get_full_ticks();
				}
			}
		break;
			
		case BLUETOOTH_WHEEL_BASE_SPEED_MODE_ID:
			wheel_base_pid_off();	//abort auto_positioning
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
			wheel_base_pid_flag=1;
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
        wheel_base_bluetooth_char_last_update = get_full_ticks();
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
	wheel_base_vel.x = wheel_base_vel.y = wheel_base_vel.w = 0;
	wheel_base_bluetooth_vel_last_update = 0;
	wheel_base_last_can_tx = 0;
	wheel_base_tx_acc();
	
	//PID on at beginning
	wheel_base_set_target_pos(wheel_base_get_target_pos());
	wheel_base_pid_on();
	
	
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
	wheel_base_target.x = x;
	wheel_base_target.y = y;
	wheel_base_target.w = w;
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

WHEEL_BASE_VEL wheel_base_get_target(void)
{
	return wheel_base_target;
}


char wheel_base_bluetooth_get_last_char(void) 
{
   return wheel_base_bluetooth_last_char;
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

int max(int a, int b)
{
	return (a > b ? a : b);
}

int min(int a, int b)
{
	return (a < b ? a : b);
}

static void vector_update(int* curr_vector, int* target_vector, const int id)
{
	const int Fibonacci[] = {1, 1, 1, 1, 2, 2, 3, 3, 5, 5};
	static int Fibonacci_id[] = {0, 0, 0};
	static int last_target[] = {0, 0, 0};
	int MAX_ID = sizeof(Fibonacci) / sizeof(Fibonacci[0]) - 1;
	int increment = Fibonacci[Fibonacci_id[id]];
	if (last_target[id] != *target_vector) {
		last_target[id] = *target_vector;
		Fibonacci_id[id] = 0;
	}

	if (*curr_vector == *target_vector) {
		Fibonacci_id[id] = 0;
		return;
	}
	/*** No Discrete jump now! ***/
	if (*curr_vector < *target_vector) {
		*curr_vector += increment;
		if (*curr_vector > *target_vector) {
			*curr_vector = *target_vector;
		}
	} else {
		*curr_vector -= increment;		
		if (*curr_vector < *target_vector) {
			*curr_vector = *target_vector;
		}
	}
//	if (Abs(*curr_vector - *target_vector) > 7) {
		Fibonacci_id[id] < MAX_ID ? ++Fibonacci_id[id] : (Fibonacci_id[id] = MAX_ID);			
//	} else {
//		Fibonacci_id[id] > 0 ? --Fibonacci_id[id] : (Fibonacci_id[id] = 0);			
//		
//	}
	
//	/*** Discrete jump ***/
//	const int jump_valued = 0;
//	// Disable discrete jump by changing it to 0
//	if (jump_valued != 0) {
//		if (*curr_vector == 0 && *curr_vector != *target_vector) {
//			// Jump for accel
//			Fibonacci_id[id] = 0;
//			if (Abs(*target_vector) < jump_valued) {
//				*curr_vector = *target_vector;
//			} else if (*target_vector > 0) {
//				*curr_vector = jump_valued;
//			} else {
//				*curr_vector = -jump_valued;
//			}
//			return;
//		} else if (*target_vector == 0) {
//			// Jump for decel
//			if (Abs(*curr_vector) < jump_valued) {
//				Fibonacci_id[id] = 0;
//				*curr_vector = 0;
//				return;
//			}
//		}
//	}
//	/*** Discrete jump ***/
//	
//	
//	/*** Fibonacci ***/
//	// jump valued + 1 1 2 3 5 or 1 1 2 3 5 8
//	int point_of_inflection = jump_valued + 12;
//	if (Abs(*curr_vector) < point_of_inflection) {
//		if (*curr_vector < *target_vector) {
//			*curr_vector += increment;
//			if (*curr_vector > *target_vector) {
//				(*curr_vector = *target_vector);
//			}
//		} else if (*curr_vector > *target_vector) {
//			*curr_vector -= increment;
//			if (*curr_vector < *target_vector) {
//				(*curr_vector = *target_vector);
//			}
//		}
//		if (Abs(*curr_vector) < jump_valued + 1) {
//			Fibonacci_id[id] = 0;
//		} else if (Abs(*curr_vector) < jump_valued + 2) {
//			Fibonacci_id[id] = 1;
//		} else if (Abs(*curr_vector) < jump_valued + 4) {
//			Fibonacci_id[id] = 2;
//		} else if (Abs(*curr_vector) < jump_valued + 7) {
//			Fibonacci_id[id] = 3;
//		} else {
//			Fibonacci_id[id] = 4;
//		}

//	}	else {
//		// Linear
//		Fibonacci_id[id] = 4;
//		if (*curr_vector < *target_vector) {
//			*curr_vector += increment;
//			if (*curr_vector >= -point_of_inflection && *target_vector <= 0) {
//				(*curr_vector = -(point_of_inflection - 1));
//			}
//			if (*curr_vector > *target_vector) {
//				*curr_vector = *target_vector;
//			}
//		} else if (*curr_vector > *target_vector) {
//			*curr_vector -= increment;
//			if (*curr_vector <= point_of_inflection && *target_vector >= 0) {
//				(*curr_vector = (point_of_inflection - 1));
//			}
//			if (*curr_vector < *target_vector) {
//				*curr_vector = *target_vector;
//			}
//		} 
//	}
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
	const int alpha = 2;
	vector_update(&(wheel_base_vel.x), &(wheel_base_target.x), 0);
	vector_update(&(wheel_base_vel.y), &(wheel_base_target.y), 1);
	if (wheel_base_target.w - wheel_base_vel.w > alpha) {
		wheel_base_vel.w += alpha;
	} else if (wheel_base_vel.w - wheel_base_target.w > alpha ) {
		wheel_base_vel.w -= alpha;
	} else {
		wheel_base_vel.w = wheel_base_target.w;
	}

	
	
	motor_set_vel(MOTOR_BOTTOM_RIGHT, (WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel.x + wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000), wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_BOTTOM_LEFT,(WHEEL_BASE_XY_VEL_RATIO * (wheel_base_vel.x - wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000), wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_TOP_LEFT,			(WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel.x - wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000), wheel_base_close_loop_flag);
	motor_set_vel(MOTOR_TOP_RIGHT,		 (WHEEL_BASE_XY_VEL_RATIO * (-wheel_base_vel.x + wheel_base_vel.y) / 1000 + WHEEL_BASE_W_VEL_RATIO * wheel_base_vel.w / 1000), wheel_base_close_loop_flag);

	wheel_base_vel_prev.x = wheel_base_vel.x;
	wheel_base_vel_prev.y = wheel_base_vel.y;
	wheel_base_vel_prev.w = wheel_base_vel.w;
	
	
	
}

bool bluetooth_is_key_release(void)
{
  return ((get_full_ticks() - wheel_base_bluetooth_char_last_update) > 200 ? true : false);
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

void wheel_base_increase_joystick_speed(void)
{
	if (wheel_base_joystick_speed<100)
		wheel_base_joystick_speed+=10;
}

void wheel_base_decrease_joystick_speed(void)
{
	if (wheel_base_joystick_speed>0)
		wheel_base_joystick_speed-=10;
}

u8 wheel_base_get_joystick_speed(void)
{
	return wheel_base_joystick_speed;
}

void is_it_moving(u8 val)
{
	is_moving = val;
}
void is_it_turning(u8 val)
{
	is_turning = val;
}
