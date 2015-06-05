#include "wheel_base_pid.h"
s32 prev_error;
s32 error;
PID wheel_base_pid = {0,0,0};

u16 integ_dw_index = 0,integ_dx_index = 0,integ_dy_index = 0;
s32 integ_dw_list[20] = {0,0,0,0,0,
												 0,0,0,0,0,
												 0,0,0,0,0,
												 0,0,0,0,0};
 
s32 integ_dx_list[10] = {0,0,0,0,0,
												 0,0,0,0,0};/*,
												 0,0,0,0,0,
												 0,0,0,0,0};*/
 
s32 integ_dy_list[10] = {0,0,0,0,0,
												 0,0,0,0,0};/*,
												 0,0,0,0,0,
												 0,0,0,0,0};*/

s32 pid_maintain_angle(void)
{
	/**ROTATIONAL PID**/
	POSITION curr_pos = {(get_pos()->x), (get_pos()->y), get_pos()->angle};	
	POSITION target = wheel_base_get_target_pos();
	
	WHEEL_BASE_VEL curr_vel = wheel_base_get_vel();
	
	s32 dw = target.angle - get_pos()->angle;
	s32 w = 0; 
	
	if (dw > 1800) {
		dw -= 3600;
	} else if (dw < -1800) {
		dw += 3600;
	}	
	
	w = dw * 3 / 10;
	//Setting velocity to be minimum magnitude of 19 and max of 50
	
	//Add integral factor
	integ_dw_list[integ_dw_index++] = dw;
	if(integ_dw_index >= 12) integ_dw_index = 0;
	s32 wi = 0;
	for (int i = 0; i < 12; i++)
	{
		wi+=integ_dw_list[i];
	}
	w+=wi/32;
	
	if (Abs(dw) >= 10) {
		w = Abs(w) < 10? w*10/Abs(w) : w;
		w = Abs(w) > 70 ? w*70/Abs(w) : w;
	} else {
		w = 0;
	}
	return w;
}



void wheel_base_pid_loop(void)
{
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	u8 speed_mode = wheel_base_get_speed_mode();
  POSITION curr_pos = {(get_pos()->x), (get_pos()->y), get_pos()->angle};	
	// For Robot D, both x and y are flipped. For Robot C, only x is flipped.
	
	POSITION target = wheel_base_get_target_pos();
	if(!wheel_base_get_pid_flag())
		return;
	/*
	//To maintain position
	target.x = curr_pos.x;
	target.y = curr_pos.y;
	
	wheel_base_set_target_pos(target);
	*/
	
	
	s32 dx = delX(curr_pos, target);
	s32 dy = delY(curr_pos, target);
	s32 dw = delW(curr_pos, target);
	
	
	/* matrix shift applied to the direction of movement */
	s32 shifted_dx = dx*int_cos(curr_pos.angle) - dy*int_sin(curr_pos.angle);
	s32 shifted_dy = dx*int_sin(curr_pos.angle) + dy*int_cos(curr_pos.angle);
	shifted_dx/=10000;	//downscale
	shifted_dy/=10000;	//downscale
	
	
	wheel_base_pid.Kp = 200/speed_mode;
	
	//xy Integral Part
	integ_dx_list[integ_dx_index++] = shifted_dx/40;
	if(integ_dx_index >= 10) integ_dx_index = 0;
	
	integ_dy_list[integ_dy_index++] = shifted_dy/40;
	if(integ_dy_index >= 10) integ_dy_index = 0;
	
	dx = shifted_dx/wheel_base_pid.Kp;
	
	for (int i = 0; i < 10; i++)
	{
		dx+=(integ_dx_list[i]);
	}
	
	dx = shifted_dx/wheel_base_pid.Kp/Abs(shifted_dx/wheel_base_pid.Kp);
	dx = Abs(shifted_dx/wheel_base_pid.Kp) < 12 ? dx*12 : shifted_dx/wheel_base_pid.Kp; 
	
	dy = shifted_dy/wheel_base_pid.Kp/Abs(shifted_dy/wheel_base_pid.Kp);
	dy = Abs(shifted_dy/wheel_base_pid.Kp) < 12 ? dy*12 : shifted_dy/wheel_base_pid.Kp; 
	
	dw = pid_maintain_angle();
	wheel_base_set_vel(dx, dy , dw); 

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

s32 delX(POSITION p1, POSITION p2)
{
	return (p2.x - p1.x);
}
s32 delY(POSITION p1, POSITION p2)
{
	return (p2.y - p1.y);
}
s32 delW(POSITION p1, POSITION p2)
{
	return (p2.angle - p1.angle);
}

