#include "wheel_base_pid.h"
s32 prev_error;
s32 error;
PID wheel_base_pid = {0,0,0};

void wheel_base_pid_loop(void)
{
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	u8 speed_mode = wheel_base_get_speed_mode();
  POSITION curr_pos = {(get_pos()->x), (get_pos()->y), get_pos()->angle};	
	// For Robot D, both x and y are flipped. For Robot C, only x is flipped.
	
	POSITION target = wheel_base_get_target_pos();
	
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
	/* untested!!*/
	s32 shifted_dx = dx*int_cos(curr_pos.angle) - dy*int_sin(curr_pos.angle);
	s32 shifted_dy = dx*int_sin(curr_pos.angle) + dy*int_cos(curr_pos.angle);
	shifted_dx/=10000;	//downscale
	shifted_dy/=10000;	//downscale
	
	
	wheel_base_pid.Kp = 200/speed_mode;
	
	
	/**ROTATIONAL PID**/
	s32 w = 0;
	dw = target.angle - get_pos()->angle;
	if( (get_pos()->angle !=  target.angle))
	{
		if((dw<= 1800 && dw >= 0) || (-dw > 1800))
		{
			//Move right
			w = 1;
		}
		else if((dw > 1800) || (-dw < 1800 && dw < 0))
		{
			//Move left
			w = -1;
		}
	}
	if(Abs(dw)> 1800) dw = dw/2;
	w = w*Abs(dw)/30;
	//Setting velocity to be minimum magnitude of 19 and max of 40
	w = Abs(w) < 19 ? w*19/Abs(w) : w;
	w = Abs(w) > 40 ? w*40/Abs(w) : w;
	wheel_base_set_vel(0, 0, w);
	//wheel_base_set_vel(shifted_dx/wheel_base_pid.Kp, shifted_dy/wheel_base_pid.Kp,w); 
	
	
	
	
	
	/*if ()
	{
		
	}*/
	
	/*
	if (dx+dy < 100 && dx+dy > -100)
	{
		wheel_base_pid_off();
	}
	
	*/
	
	/*
	POSITION proportion = {target_pos.x-(-get_pos()->x), target_pos.y-get_pos()->y, target_pos.angle-get_pos()->angle};
	s32 positioning_magnitude_x = proportion.x/Abs(proportion.x) * wheel_base_speed_mode*2;
	s32 positioning_magnitude_y = proportion.y/Abs(proportion.y) * wheel_base_speed_mode*2;
	wheel_base_set_vel(positioning_magnitude_x+proportion.x/200*wheel_base_speed_mode, positioning_magnitude_y+proportion.y/200*wheel_base_speed_mode,0);
	tft_prints(0, 4, "P :(%3d,%3d,%3d)", proportion.x, proportion.y, proportion.angle);
	tft_prints(0, 5, "P enabled = %d", auto_positioning_enabled);
	tft_update();
	
d 
	if (proportion.x+proportion.y < 200 && proportion.x+proportion.y > -200)
	{
		wheel_base_pid_off();
	}
	*/
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

