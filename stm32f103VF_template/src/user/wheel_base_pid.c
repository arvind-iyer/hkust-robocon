#include "wheel_base_pid.h"
#include "gyro.h"

static PID_output_vel output = { 0, 0, 0};
static PID_error PID_err = {0,0,0};
static PID_para PID_val = {10,0,0};
static PID_error pre_err = {0,0,0};
volatile s32 cur_x = 0, cur_y =0, cur_w =0;
volatile POSITION cur_position = {0};
static POSITION target_pos = {0};
static WHEEL_BASE_VEL vel = {0};

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

void wheel_base_pid_update(void)
{
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	
	wheel_base_pid_on();
	// get position
	cur_position = get_position();
	cur_x = cur_position.x;
	cur_y = cur_position.y;
	cur_w = cur_position.angle;
	
	/***error update***/
	//  x
	PID_err.diff.x = wheel_base_get_target_pos().x - cur_x;
	PID_err.derivative.x = (PID_err.diff.x - pre_err.diff.x) / time_interval ;
	PID_err.intergral.x =PID_err.intergral.x + PID_err.diff.x * time_interval;

	//  y
	PID_err.diff.y = wheel_base_get_target_pos().y -cur_position.y;
	PID_err.derivative.y = (PID_err.diff.y - pre_err .diff.y  ) / time_interval ;
	PID_err.intergral.y =PID_err.intergral.y + PID_err.diff.y * time_interval;
	
	//  w ---0
	PID_err.diff.w = -cur_position.angle;
	PID_err.derivative.w = (PID_err.diff.w - pre_err .diff.w  ) / time_interval ;
	PID_err.intergral.w = PID_err.intergral.w + PID_err.diff.w * time_interval;
	
	/*** set velocity  ***/
	if(PID_err.diff.x > 5000){
		vel.x = (PID_err.diff.x * PID_val.Kp_xy + PID_err.derivative.x * PID_val.Kd_xy + PID_err.intergral.x * PID_val.Ki_xy)/2500;
	}else if (PID_err.diff.x > 500){
		vel.x = PID_err.diff.x * PID_val.Kp_xy / 500 ;
	}else if (PID_err.diff.x > 50){
		vel.x = PID_err.diff.x * PID_val.Kp_xy / 50;
	}else{
		vel.x = PID_err.diff.x * PID_val.Kp_xy;
	}
	
	if(PID_err.diff.y > 5000){
		vel.y = (PID_err.diff.y * PID_val.Kp_xy + PID_err.derivative.y * PID_val.Kd_xy + PID_err.intergral.y * PID_val.Ki_xy)/5000;
	}else if (PID_err.diff.y > 500){
		vel.y = PID_err.diff.y * PID_val.Kp_xy / 500 ;
	}else if (PID_err.diff.y > 50){
		vel.y = PID_err.diff.y * PID_val.Kp_xy / 50;
	}

	vel.w = (PID_err.diff.w * PID_val.Kp_w + PID_err.derivative.w * PID_val.Kd_w + PID_err.intergral.w * PID_val.Ki_w)/1000;
	wheel_base_set_vel (vel.x ,vel.y ,vel.w);
	
	// update error
	pre_err = PID_err;
	
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



