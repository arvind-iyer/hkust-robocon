#include "wheel_base_pid.h"
#include "gyro.h"

static PID_output_vel output = { 0, 0, 0};
static PID_error PID_err = {0,0,0};
static PID_para PID_val = {1,1,0};
static PID_error pre_err = {0,0,0,0,1,0};
volatile s32 cur_x = 0, cur_y =0, cur_w =0;
volatile POSITION cur_position = {0};
static POSITION target_pos = {0};
static WHEEL_BASE_VEL vel = {0};
static POSITION target_record = {0};
static u8 target_changed_flag = 0;
static u8 count =0;

void set_PID_val (PID_para para){
//xy
	PID_val.Kp_xy = para.Kp_xy;
	PID_val.Ki_xy = para.Ki_xy;
	PID_val.Kd_xy = para.Kd_xy;
//w
	PID_val.Kp_w = para.Kp_w;
	PID_val.Ki_w = para.Ki_w;
	PID_val.Kd_w = para.Kd_w;
} 


//  P + I 

void wheel_base_pid_update(void)
{
	 	
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */

	//target update
	target_pos = wheel_base_get_target_pos();
	
	// get position
	cur_position = get_position();
	cur_x = cur_position.x;
	cur_y = cur_position.y;
	cur_w = cur_position.angle;
	
	// check target change
	if(target_record.x == target_pos.x && target_record.y == target_pos.y &&target_record.angle == target_pos.angle){
		target_changed_flag = 0;
	}else {
		target_changed_flag = 1;
		count++;
	}
	
	/***error update***/
	//  x
	PID_err.diff.x = wheel_base_get_target_pos().x - cur_x;
	PID_err.derivative.x = (PID_err.diff.x - pre_err.diff.x) / time_interval ;
	PID_err.intergral.x =PID_err.intergral.x + PID_err.diff.x * time_interval;

	//  y
	PID_err.diff.y = wheel_base_get_target_pos().y -cur_position.y;
	PID_err.derivative.y = (PID_err.diff.y - pre_err .diff.y  ) / time_interval ;
	PID_err.intergral.y =PID_err.intergral.y + PID_err.diff.y * time_interval;
	
	//  w  angle : 0 - 3599
	PID_err.diff.w = wheel_base_get_target_pos().angle - cur_position.angle;
	if(PID_err.diff.w > 1800){
		PID_err.diff.w = - 3600 + PID_err.diff.w;
	} else if (PID_err.diff.w < -1800){
		PID_err.diff.w = 3600 + PID_err.diff.w;
	}
	
	PID_err.derivative.w = (PID_err.diff.w - pre_err .diff.w  ) / time_interval ;
	PID_err.intergral.w = PID_err.intergral.w + PID_err.diff.w * time_interval;
	
	/***     use function of distance to change the parameter and velocity       ***/
	PID_val.Kd_xy = 10;
	
	// adding line correction
	
	
	
	//x
	if(PID_err.diff.x > 100 || PID_err.diff.x < -100){
	vel.x = (PID_err.diff.x * PID_val.Kp_xy + PID_err.derivative.x * PID_val.Kd_xy + PID_err.intergral.x * PID_val.Ki_xy)/15;
	}else if(PID_err.diff.x > 30 ){
		vel.x =	10;
	} else if (PID_err.diff.x < -30){
		vel.x = -10;
	}else {
		vel.x = 0;
	}
	
// y
	if(PID_err.diff.y > 100 || PID_err.diff.y < -100){
	vel.y = (PID_err.diff.y * PID_val.Kp_xy + PID_err.derivative.y * PID_val.Kd_xy + PID_err.intergral.y * PID_val.Ki_xy)/15;
	}else if(PID_err.diff.y > 30 ){
		vel.y = 10;
	} else if (PID_err.diff.y < -30){
		vel.y = -10;
	}else {
		vel.y = 0;
	}
	
	vel .w = PID_err.diff.w * PID_val.Kp_w / 30; 
	
	// w
	if(PID_err.diff.w >100 || PID_err.diff.w < -100){
	}else if (PID_err.diff.w >30){
		vel .w = 10 ;
	}else if (PID_err.diff.w <-30) {
		vel .w = - 10 ;
	}else {
		vel.w = 0 ;
	}
	
//limit vel
		vel.x = LIMITED (	vel.x, -xy_vel_limit, xy_vel_limit );
		vel.y = LIMITED (	vel.y, -xy_vel_limit, xy_vel_limit );
	  vel.w = LIMITED ( vel.w, -w_vel_limit,  w_vel_limit ) ;
	
	//x
//	if(PID_err.diff.x > 5000	||	PID_err.diff.x <- 5000){
//		vel.x = (PID_err.diff.x * PID_val.Kp_xy + PID_err.derivative.x * PID_val.Kd_xy + PID_err.intergral.x * PID_val.Ki_xy)/2000;
//	}else if (PID_err.diff.x > 2000 ||	PID_err.diff.x <- 2000	){
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 100  ;
//	}else if (PID_err.diff.x > 600 ||	PID_err.diff.x <- 600){
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 30;
//	}else if (PID_err.diff.x > 200 ||	PID_err.diff.x <- 200){
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 8;
//	}else if (PID_err.diff.x > 60 ||	PID_err.diff.x <- 60){
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 3;
//	}else if (PID_err.diff.x > 20 ||	PID_err.diff.x <- 20){
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 2;
//	}else {
//		vel.x = PID_err.diff.x * PID_val.Kp_xy / 3;
//	}
//	

//	//y
//	if(PID_err.diff.y > 5000	||	PID_err.diff.y <- 5000){
//		vel.y = (PID_err.diff.y * PID_val.Kp_xy + PID_err.derivative.y * PID_val.Kd_xy + PID_err.intergral.y * PID_val.Ki_xy)/2000;
//	}else if (PID_err.diff.y > 2000 ||	PID_err.diff.y <- 2000	){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 100  ;
//	}else if (PID_err.diff.y > 600 ||	PID_err.diff.y <- 600){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 30;
//	}else if (PID_err.diff.y > 200 ||	PID_err.diff.y <- 200){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 8;
//	}else if (PID_err.diff.y > 60 ||	PID_err.diff.y <- 60){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 3;
//	}else if (PID_err.diff.y > 20 ||	PID_err.diff.y <- 20){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 2;
//	}else {
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 3;
//	}

	// w
/*	if(PID_err.diff.w >0){
		vel.w = (PID_err.diff.w % 3600) * PID_val.Kp_w ;
	}
	*/
	
	/*** set velocity  ***/
	
//	
//	if(PID_err.diff.y > 5000){
//		vel.y = (PID_err.diff.y * PID_val.Kp_xy + PID_err.derivative.y * PID_val.Kd_xy + PID_err.intergral.y * PID_val.Ki_xy)/5000;
//	}else if (PID_err.diff.y > 500){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 500 ;
//	}else if (PID_err.diff.y > 50){
//		vel.y = PID_err.diff.y * PID_val.Kp_xy / 50;
//	}

	if(wheel_base_get_pid_flag()){
		motor_set_acceleration(MOTOR_BOTTOM_RIGHT, pid_acc);
		wheel_base_set_vel (vel.x ,vel.y ,vel.w);
	}
	// update error
	pre_err = PID_err ;
	target_record = target_pos;
}

s32 get_vx(void){
	return vel.x;
}
s32 get_vy(void){
	return vel.y;
}
s32 get_vw(void){
	return vel.w;
}

s32 get_PID_err_diff_x (void){
	return PID_err.diff.x;
}

s32 get_PID_err_diff_y (void){
	return PID_err.diff.y;
}

s32 get_PID_err_diff_w (void){
	return PID_err.diff.w;
}

//s16 get_slope_num (s16 x, s16 y){
//	return y;
//}

//s16 get_slope_deno (s16 x, s16 y){
//		return x;
//}
// 

s8 get_target_changed_flag(void){
	return target_changed_flag;
}

s8 get_change_count (void){
	return count;
}
