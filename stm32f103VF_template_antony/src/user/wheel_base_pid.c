#include "wheel_base_pid.h"

s32 pid_ratio_multiplier = 0;
s32 pid_ratio_divider = 1;

void kp_higher(void) { $kp[DIR_X]+=5; }
void kp_lower(void) { $kp[DIR_X]-=5; }
void ki_higher(void) { $ki[DIR_X]+=5; }
void ki_lower(void) { $ki[DIR_X]-=5; }
void kd_higher(void) { $kd[DIR_X]+=5; }
void kd_lower(void) { $kd[DIR_X]-=5; }
void kpa_higher(void) { $kp[ANGLE]+=5; }
void kpa_lower(void) { $kp[ANGLE]-=5; }
s32 get_kpx(void) { return $kp[DIR_X]; }
s32 get_kix(void) { return $ki[DIR_X]; }
s32 get_kdx(void) { return $kd[DIR_X]; }
u8 get_speed_mode(void) { return $speed_mode; }
s16 get_angle_error(void) { return $error.angle; }
s32 get_kpa(void) { return $kp[ANGLE]; }
WHEEL_BASE_VEL get_vel_limit(void) { return $vel_limit; }

/* void gyro_set_zero(void) {
	gyro_pos_set(0, 0, 0);
} */

void wheel_base_pid_init(void) {
	// register_special_char_function('P', kp_higher);
	// register_special_char_function('O', ki_higher);
	// register_special_char_function('I', kd_higher);
	
	// register_special_char_function('p', kp_lower);
	// register_special_char_function('o', ki_lower);
	// register_special_char_function('i', kd_lower);
	//register_special_char_function('g', gyro_set_zero);
	
	$vel_limit.x = VELOCITY_LIMIT_X;
	$vel_limit.y = VELOCITY_LIMIT_Y;
	$vel_limit.w = VELOCITY_LIMIT_W;
}

WHEEL_BASE_VEL wheel_base_pid_current_vel(void) {
	return $set_vel;
}

WHEEL_BASE_VEL wheel_base_pid_unlimit_vel(void) {
	return $unlimit_vel;
}

void set_pid_limit_ratio(s32 vel_x, s32 vel_y) {
	vel_x = ABS(vel_x);
	vel_y = ABS(vel_y);
	if(vel_x > $vel_limit.x && vel_y > $vel_limit.y) {
		if($vel_limit.x * vel_y > $vel_limit.y * vel_x) {
			pid_ratio_multiplier = $vel_limit.y;
			pid_ratio_divider = vel_y;
		} else {
			pid_ratio_multiplier = $vel_limit.x;
			pid_ratio_divider = vel_x;
		}
	} else if(vel_x > $vel_limit.x) {
		pid_ratio_multiplier = $vel_limit.x;
		pid_ratio_divider = vel_x;
	} else if(vel_y > $vel_limit.y) {
		pid_ratio_multiplier = $vel_limit.y;
		pid_ratio_divider = vel_y;
	} else {
		pid_ratio_multiplier = 1;
		pid_ratio_divider = 1;
	}
}

void wheel_base_pid_update(void) {
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	
	if( wheel_base_get_pid_flag() == 0 ) {
		wheel_base_set_target_pos( *get_pos() );
		$in.angle = 0;
		return;
	}
	
	$current = *get_pos();
	$target  = wheel_base_get_target_pos();
	
	$last_error = $error;
	$error.x = $target.x - $current.x;
	$error.y = $target.y - $current.y;
	$error.angle = ANGLE_PN1800($target.angle - $current.angle);
	
	$de.x = $error.x - $last_error.x;
	$de.y = $error.x - $last_error.x;
	$de.angle = $error.angle - $last_error.angle;
	
	/* $in.x += $error.x;
	$in.y += $error.y;
	$in.angle += $error.angle; DANGEROUS */
	
	$in.x = 0;
	$in.y = 0;
	$in.angle = 0;
	
	$set_vel.x = ($kp[DIR_X] * $error.x + $ki[DIR_X] * $in.x + $kd[DIR_X] * $de.x) / K_DIVIDER;
	$set_vel.y = ($kp[DIR_Y] * $error.y + $ki[DIR_Y] * $in.y + $kd[DIR_Y] * $de.y) / K_DIVIDER;
	$set_vel.w = ($kp[ANGLE] * $error.angle + $ki[ANGLE] * $in.angle + $kd[ANGLE] * $de.angle) / K_DIVIDER;
	
	$speed_mode = wheel_base_get_speed_mode();
	//
	if($speed_mode >= 9) {
		$speed_mode = 9;
	}
	//
	$set_vel.x = $set_vel.x * PID_SPEED_MODE[$speed_mode] / 100;
	$set_vel.y = $set_vel.y * PID_SPEED_MODE[$speed_mode] / 100;
	$set_vel.w = $set_vel.w * PID_SPEED_MODE[$speed_mode] / 100;
	
	$unlimit_vel = $set_vel;
	
	$vel_limit.x = VELOCITY_LIMIT_X * PID_SPEED_MODE[$speed_mode] / 100;
	$vel_limit.y = VELOCITY_LIMIT_Y * PID_SPEED_MODE[$speed_mode] / 100;
	$vel_limit.w = VELOCITY_LIMIT_W * PID_SPEED_MODE[$speed_mode] / 100;
	set_pid_limit_ratio($set_vel.x, $set_vel.y);
	
	$set_vel.x = $set_vel.x * pid_ratio_multiplier / pid_ratio_divider;
	$set_vel.y = $set_vel.y * pid_ratio_multiplier / pid_ratio_divider;
	
	if( ABS($set_vel.x) < POSITION_FIXING_SPEED && $unlimit_vel.x != 0 ) {
		if( ABS($error.x) > POSITION_FIXING_THRESHOLD ) {
			$set_vel.x = SIGN($unlimit_vel.x) * POSITION_FIXING_SPEED;
		}
	}
	
	if( ABS($set_vel.y) < POSITION_FIXING_SPEED && $unlimit_vel.y != 0) {
		if( ABS($error.y) > POSITION_FIXING_THRESHOLD ) {
			$set_vel.y = SIGN($unlimit_vel.y) * POSITION_FIXING_SPEED;
		}
	}
	
	// Ensure the velocities do not exceed the limit
	$set_vel.x = LIMITED($set_vel.x, -$vel_limit.x, $vel_limit.x);
	$set_vel.y = LIMITED($set_vel.y, -$vel_limit.y, $vel_limit.y);
	$set_vel.w = LIMITED($set_vel.w, -$vel_limit.w, $vel_limit.w);
	
	wheel_base_set_vel($set_vel.x, $set_vel.y, 0);
}


