#include "wheel_base_pid.h"
s32 prev_error;
s32 error;
PID wheel_base_pid = {0,0,0};
void wheel_base_pid_loop(void)
{
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
  
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
s32 delA(POSITION p1, POSITION p2)
{
	return (p2.angle - p1.angle);
}

