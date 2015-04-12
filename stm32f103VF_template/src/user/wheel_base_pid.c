#include "wheel_base_pid.h"
#include "special_char_handler.h"
#include "stdbool.h"
#include "xbc_button.h"

#define STEADY_STATE_ERROR_THRESHOLD 10
#define T_STEADY_STATE_ERROR_THRESHOLD 5

#define STEADY_STATE_TIME_THRESHOLD 200

#define ERROR_THRESHOLD 800
#define INTEGRAL_THRESHOLD 1000
#define MOTOR_MAX_SPEED 1300

#define T_ERROR_THRESHOLD 400
#define T_MOTOR_MAX_SPEED 500

// gyro variables
const static int GYRO_TICKS_TIMEOUT = 2000;
static bool gyro_calibrated = false;

static int C_PR = 234;
static int C_IN = 0;
static int C_DE = 0;

static int C_T_PR = 600;
static int C_T_IN = 0;
static int C_T_DE = 0;

typedef struct {
	s32 current_error, p_error, i_error, d_error;
} PID_OUTPUT;

static POSITION end_pos = {0, 0, 0};

static PID_OUTPUT x_coord_pid = {0, 0, 0, 0}, 
									y_coord_pid = {0, 0, 0, 0}, 
									t_coord_pid = {0, 0, 0, 0},
									// for resetting PID to zero
									reset = {0, 0, 0, 0};

static s32 x_speed = 0, y_speed = 0, t_speed = 0;

static s32 x_steady_state_timer = 0;
static s32 y_steady_state_timer = 0;
static s32 t_steady_state_timer = 0;
									
static bool pid_locked = false;

static u8 der_count_up = 0;
static u8 der_count_down = 0;
static u8 int_count_up = 0;
static u8 int_count_down = 0;
static u8 pro_count_up = 0;
static u8 pro_count_down = 0;

static void decrease_der(void)
{
	if (der_count_down == 9) {
		--C_DE;
		der_count_down = 0;
	} else {
		++der_count_down;
	}
}

static void increase_der(void)
{
	if (der_count_up == 9) {
		++C_DE;
		der_count_up = 0;
	} else {
		++der_count_up;
	}
}
									
static void decrease_int(void)
{
	if (int_count_down == 9) {
		--C_IN;
		int_count_down = 0;
	} else {
		++int_count_down;
	}
}

static void increase_int(void)
{
	if (int_count_up == 9) {
		++C_IN;
		int_count_up = 0;
	} else {
		++int_count_up;
	}
}
									
static void decrease_prop(void)
{
	if (pro_count_down == 9) {
		--C_PR;
		pro_count_down = 0;
	} else {
		++pro_count_down;
	}
}

static void increase_prop(void)
{
	if (pro_count_up == 9) {
		++C_PR;
		pro_count_up = 0;
	} else {
		++pro_count_up;
	}
}

// reset gyro to 0,0
static void reset_gyro(void)
{
	gyro_pos_set(0, 0, 0);
	POSITION target_pos = {0, 0, 0};
	wheel_base_set_target_pos(target_pos);
}

// calibrate gyro to the T shaped point on front side of game field
void set_starting_pos(void)
{
	#warning hard_coded!
	gyro_calibrated = gyro_pos_set(0, 4700, 0) || 1;
	if (gyro_calibrated) {
		POSITION target_pos = {0, 4700, 0};
		wheel_base_set_target_pos(target_pos);
	}
}

// function to set PID directly to serving position
void set_serving_pos(void)
{
	POSITION target_pos = {343, 4293, 3500};
	wheel_base_set_target_pos(target_pos);
}

// returning pos is the position to return the ball on the non-yellow zone side of the field
void set_returning_pos(void)
{
	POSITION target_pos = {-1373, 2404, 0};
	wheel_base_set_target_pos(target_pos);
}

// function to move back after serve
void set_after_serve_pos(void)
{
	POSITION target_pos = {1000, 500, 0};
	wheel_base_set_target_pos(target_pos);
}

void wheel_base_pid_init(void)
{
	register_special_char_function('r', decrease_prop);
	register_special_char_function('t', increase_prop);
	register_special_char_function('f', decrease_int);
	register_special_char_function('g', increase_int);
	register_special_char_function('v', decrease_der);
	register_special_char_function('b', increase_der);
	register_special_char_function('/', reset_gyro);
	register_special_char_function('\t', set_starting_pos);
	register_special_char_function('x', set_serving_pos);
}

u8 get_pid_stat(void)
{
	return pid_locked;
}

int get_x_speed(void)
{
	return x_speed;
}

int get_y_speed(void)
{
	return y_speed;
}

int get_t_speed(void)
{
	return t_speed;
}

int get_prop(void)
{
	return C_PR;
}

int get_int(void)
{
	return C_IN;
}

int get_der(void)
{
	return C_DE;
}

void update_pid(s32 current_error, PID_OUTPUT* input_pid)
{
	// Calculate proportional
	input_pid->p_error = current_error;
	
	// Calculate integral
	input_pid->i_error += current_error / 10;
	
	// prevent integral from growing too large
	if (input_pid->i_error > INTEGRAL_THRESHOLD) {
		input_pid->i_error = INTEGRAL_THRESHOLD;
	}
	
	// Calculate derivative
	input_pid->d_error = current_error - input_pid->current_error;
	
	// Update error stored in PID
	input_pid->current_error = current_error;
}

// Reset all error calculations
void reset_pid(void)
{
	x_coord_pid = reset;
	y_coord_pid = reset;
	t_coord_pid = reset;
}

void wheel_base_pid_update(void)
{
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	
	// auto gyro calibration code
	if (!gyro_calibrated && get_full_ticks() > GYRO_TICKS_TIMEOUT) {
		#warning hard_coded!
		gyro_calibrated = gyro_pos_set(0, 4700, 0) || 1; 
		
		if (gyro_calibrated) {
			POSITION target_pos = {0, 4700, 0};
			wheel_base_set_target_pos(target_pos);
		}
	}
	
	// PID mode check
	if (wheel_base_get_pid_flag() == 1 && gyro_calibrated) {
		
		// do not calculate PID if adjusting manually after PID is reached
		if ((get_full_ticks() - wheel_base_get_last_manual_timer()) < BLUETOOTH_WHEEL_BASE_TIMEOUT + 200 ||
			(get_full_ticks() - xbc_get_received_nonzero_speed_timer()) < BLUETOOTH_WHEEL_BASE_TIMEOUT + 200) {
			wheel_base_set_target_pos(*get_pos());
			reset_pid();
			return;
		}
		
		// update position variables and reset PID if target position is changed
		if (end_pos.x != wheel_base_get_target_pos().x || end_pos.y != wheel_base_get_target_pos().y || end_pos.angle != wheel_base_get_target_pos().angle) {
			end_pos = wheel_base_get_target_pos();
			reset_pid();
			pid_locked = false;
		}
		
		s32 x_error = end_pos.x - get_pos()->x, y_error = end_pos.y - get_pos()->y, t_error = end_pos.angle - get_pos()->angle;
		bool x_lock = false, y_lock = false, t_lock = false;
		
		// update steady state timer if not in threshold
		if (x_error > STEADY_STATE_ERROR_THRESHOLD || x_error < -STEADY_STATE_ERROR_THRESHOLD)
		{
			x_steady_state_timer = get_full_ticks();
		}
		if (y_error > STEADY_STATE_ERROR_THRESHOLD || y_error < -STEADY_STATE_ERROR_THRESHOLD)
		{
			y_steady_state_timer = get_full_ticks();
		}
		if (t_error > T_STEADY_STATE_ERROR_THRESHOLD || t_error < -T_STEADY_STATE_ERROR_THRESHOLD)
		{
			t_steady_state_timer = get_full_ticks();
		}
		
		// when timer is expired, then we reset PID and adjust the position
		if (get_full_ticks() - x_steady_state_timer > STEADY_STATE_TIME_THRESHOLD)
		{
			end_pos.x = get_pos()->x;
			x_coord_pid = reset;
			x_lock = true;
		}
		if (get_full_ticks() - y_steady_state_timer > STEADY_STATE_TIME_THRESHOLD)
		{
			end_pos.y = get_pos()->y;
			y_coord_pid = reset;
			y_lock = true;
		}
		if (get_full_ticks() - t_steady_state_timer > STEADY_STATE_TIME_THRESHOLD)
		{
			end_pos.angle = get_pos()->angle;
			t_coord_pid = reset;
			t_lock = true;
		}
		// disable angle pid
		// t_lock = true;
		
		// update lock PID
		if (x_lock && y_lock && t_lock)
		{
			pid_locked = true;
		} else {
			pid_locked = false;
		}
		
		// calculate shortest angle error
		if (t_error > 1800) {
			t_error -= 3600;
		} else if (t_error < -1800) {
			t_error += 3600;
		}
		
		s32 x_max_speed, y_max_speed;
		if (abs(x_error) > abs(y_error))
		{
			x_max_speed = MOTOR_MAX_SPEED;
			y_max_speed = MOTOR_MAX_SPEED * abs(y_error) / abs(x_error);
		}
		else
		{
			x_max_speed = MOTOR_MAX_SPEED * abs(x_error) / abs(y_error);
			y_max_speed = MOTOR_MAX_SPEED;
		}
		
		// x coordinate PID update
		if (x_error <= ERROR_THRESHOLD && x_error >= -ERROR_THRESHOLD) {
			// Update PID
			update_pid(x_error, &x_coord_pid);
			// Set speed according to PID
			x_speed = C_PR * x_coord_pid.p_error / 100 + C_IN * x_coord_pid.i_error / 100 + C_DE * x_coord_pid.d_error / 100;
		} else if (x_error > ERROR_THRESHOLD) {
			// Error too large, just set motor to positive max
			x_speed = x_max_speed;
		} else {
			// Error too large, just set motor to negative max
			x_speed = -x_max_speed;
		}
		
		// y coordinate PID update
		
		// Update PID
		update_pid(y_error, &y_coord_pid);
		// Set speed according to PID
		y_speed = C_PR * y_coord_pid.p_error / 100 + C_IN * y_coord_pid.i_error / 100 + C_DE * y_coord_pid.d_error / 100;

		if (y_speed > y_max_speed) {
			// Error too large, just set motor to positive max
			y_speed = y_max_speed;
			// Reset integral
			y_coord_pid.i_error = 0;
		} else if (y_speed < -y_max_speed) {
			// Error too large, just set motor to negative max
			y_speed = -y_max_speed;
			// Reset integral
			y_coord_pid.i_error = 0;
		}
		
		// angle PID update
		
		// Update PID
		update_pid(t_error, &t_coord_pid);
		// Set speed according to PID
		t_speed = C_T_PR * t_coord_pid.p_error / 100 + C_T_IN * t_coord_pid.i_error / 100 + C_T_DE * t_coord_pid.d_error / 100;
		
		if (t_speed > T_MOTOR_MAX_SPEED) {
			// Speed too positive, just set motor to positive max
			t_speed = T_MOTOR_MAX_SPEED;
			// Reset integral
			t_coord_pid.i_error = 0;
		} else if (t_speed < -T_MOTOR_MAX_SPEED) {
			// Speed too negative, just set motor to negative max
			t_speed = -T_MOTOR_MAX_SPEED;
			// Reset integral
			t_coord_pid.i_error = 0;
		}
		
		// scale speed with angle
		
		int x_final_speed = (x_speed * int_cos(get_pos()->angle) - y_speed * int_sin(get_pos()->angle)) / 10000;
		int y_final_speed = (y_speed * int_cos(get_pos()->angle) + x_speed * int_sin(get_pos()->angle)) / 10000;
		int t_final_speed = t_speed;
		
		u8 speed_ratio;
		// scale speeds using speed mode
		if (SPEED_MODES[wheel_base_get_speed_mode()]) {
			speed_ratio = 60;
		} else {
			speed_ratio = 0;
		}

		// set velocity
		wheel_base_set_vel(x_final_speed * speed_ratio / 1000, y_final_speed * speed_ratio / 1000, t_final_speed * speed_ratio / 1000);
	}
}
