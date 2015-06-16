#include "wheel_base_pid.h"
#include "gyro.h"

u16 integ_dw_index = 0;
s32 integ_dw_list[20] = {0,0,0,0,0,
                         0,0,0,0,0,
                         0,0,0,0,0,
                         0,0,0,0,0};

s32 pid_maintain_angle(void) {
	POSITION curr_pos = {get_pos()->x, get_pos()->y, get_pos()->angle};
	POSITION target = wheel_base_get_target_pos();
	
	WHEEL_BASE_VEL curr_vel = wheel_base_get_vel();
	
	s32 dw = target.angle - curr_pos.angle;
	s32 w = 0;
	if (dw > 1800) {
		dw -= 3600;
	} else if (dw < -1800) {
		dw += 3600;
	}
	
	// omega due to angle error (P value)
	w = dw * 3 / 10;
	
	// Add integral factor
	integ_dw_list[integ_dw_index++] = dw;
	if (integ_dw_index >= 12)
		integ_dw_index = 0;
	s32 wi = 0;
	for (int i=0; i<12; i++) {
		wi += integ_dw_list[i];
	}
	// omega due to I value. (w is PI control now)
	w += wi / 32;
	
	// Dead zone and velocity limit.
	if (Abs(dw) >= 10) {
		w = Abs(w) < 10 ? w*10/Abs(w) : w;
		w = Abs(w) > 70 ? w*70/Abs(w) : w;
	} else {
		w = 0;
	}
	return w;
}

void wheel_base_pid_update(void) {
  /** TODO: Code the auto PID **/
  /** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
}
