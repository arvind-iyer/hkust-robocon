#include "wheel_base_pid.h"
#include "gyro.h"

static PID_output_vel output = { 0, 0, 0};
static PID_error PID_err = {0,0,0};
static PID_error pre_err = {0,0,0,0,1,0};
volatile POSITION current_pos = {0};
static POSITION target_pos = {0};
static WHEEL_BASE_VEL vel = {0};
static POSITION target_record = {0};
static u8 target_changed_flag = 0;
static u8 count =0;

static s32 relative_velocity_x;
static s32 relative_velocity_y;
static s32 relative_velocity_w;

//  P + I 


void calculate_pid_limit_ratio(s32 vel_x, s32 vel_y, s32 vel_limit_x, s32 vel_limit_y) {
	vel_x = abs(vel_x);
	vel_y = abs(vel_y);
	if(vel_x > vel_limit_x && vel_y > vel_limit_y) {
		if(vel_limit_x * vel_y > vel_limit_y * vel_x) {
			pid_ratio_multiplier = vel_limit_y;
			pid_ratio_divider = vel_y;
		} else {
			pid_ratio_multiplier = vel_limit_x;
			pid_ratio_divider = vel_x;
		}
	} else if(vel_x > vel_limit_x) {
		pid_ratio_multiplier = vel_limit_x;
		pid_ratio_divider = vel_x;
	} else if(vel_y > vel_limit_y) {
		pid_ratio_multiplier = vel_limit_y;
		pid_ratio_divider = vel_y;
	} else {
		pid_ratio_multiplier = 1;
		pid_ratio_divider = 1;
	}
}

void wheel_base_pid_update(void) {
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */

	if(wheel_base_get_pid_flag()) {
			 
		// target update
		target_pos = wheel_base_get_target_pos();
		current_pos = *get_pos();
		
		if(abs(target_pos.x - current_pos.x) < 10 && abs(target_pos.y - current_pos.y) < 10 && abs(target_pos.angle - current_pos.angle) < 10) {
				wheel_base_set_vel(0, 0, 0);
				return;
		}
		
		// check target change
		if(target_record.x == target_pos.x && target_record.y == target_pos.y && target_record.angle == target_pos.angle) {
			target_changed_flag = 0;
		} else {
			target_changed_flag = 1;
			count++;
		}
		
		/***error update***/
		// x
		PID_err.diff.x = wheel_base_get_target_pos().x - current_pos.x;
		//PID_err.derivative.x = (PID_err.diff.x - pre_err.diff.x) / time_interval ;
		//PID_err.integral.x = 0;
		
		// y
		PID_err.diff.y = wheel_base_get_target_pos().y - current_pos.y;
		//PID_err.derivative.y = (PID_err.diff.y - pre_err .diff.y  ) / time_interval ;
		//PID_err.integral.y = 0;
		
		// w angle: 0 - 3599
		PID_err.diff.w = wheel_base_get_target_pos().angle - current_pos.angle;
		if(PID_err.diff.w > 1800) {
			PID_err.diff.w = - 3600 + PID_err.diff.w;
		} else if (PID_err.diff.w < -1800) {
			PID_err.diff.w = 3600 + PID_err.diff.w;
		}
		
		PID_err.derivative.w = (PID_err.diff.w - pre_err .diff.w ) / time_interval ;
		//PID_err.integral.w = PID_err.integral.w + PID_err.diff.w * time_interval;
		PID_err.integral.w = 0;
		
		// P only
		vel.x = PID_err.diff.x * PID_XY_P / 10000;
		vel.y = PID_err.diff.y * PID_XY_P / 10000;
		vel.w = PID_err.diff.w * PID_W_P / 10000;
		
		calculate_pid_limit_ratio(vel.x, vel.y, xy_vel_limit, xy_vel_limit);
		vel.x = vel.x * pid_ratio_multiplier / pid_ratio_divider;
		vel.y = vel.y * pid_ratio_multiplier / pid_ratio_divider;
		
		// limit vel
		vel.x = LIMITED (	vel.x, -xy_vel_limit, xy_vel_limit );
		vel.y = LIMITED (	vel.y, -xy_vel_limit, xy_vel_limit );
		vel.w = LIMITED ( vel.w, -w_vel_limit,  w_vel_limit ) ;
		
		// motor_set_acceleration(MOTOR_BOTTOM_RIGHT, pid_acc);
		relative_velocity_x = vel.x * int_cos(get_pos()->angle) / 10000 - vel.y * int_sin(get_pos()->angle) / 10000;
		relative_velocity_y = vel.y * int_cos(get_pos()->angle) / 10000 + vel.x * int_sin(get_pos()->angle) / 10000;
		relative_velocity_w = vel.w;
		wheel_base_set_vel(relative_velocity_x, relative_velocity_y, relative_velocity_w);
	} else {
		wheel_base_set_target_pos( *get_pos() );
	}
	
	// update error
	pre_err = PID_err ;
	target_record = target_pos;
}

s32 get_vx(void) { return relative_velocity_x; }
s32 get_vy(void) { return relative_velocity_y; }
s32 get_vw(void) { return relative_velocity_w; }

s32 get_PID_err_diff_x (void){
	return PID_err.diff.x;
}

s32 get_PID_err_diff_y (void){
	return PID_err.diff.y;
}

s32 get_PID_err_diff_w (void){
	return PID_err.diff.w;
}

s8 get_target_changed_flag(void){
	return target_changed_flag;
}

s8 get_change_count (void){
	return count;
}
