#include "angle_lock.h"
#include "gyro.h"

s16 angle_prev = 0;
s16 angle_target = 0;
u32 angle_lock_ignore_until_ticks = 0;

s16 angle_lock_mv = 0;

void angle_lock_init(void)
{
	angle_lock_mv = 0;
	angle_target = get_pos()->angle; 
}


bool angle_lock_ignored(void)
{
	return !gyro_get_available() || get_full_ticks() <= angle_lock_ignore_until_ticks;
}

void angle_lock_update(void)
{
	if (!angle_lock_ignored()) {
		if (angle_lock_ignore_until_ticks) {
			// Angle lock resumes
			angle_lock_ignore_until_ticks = 0;
			angle_target = get_pos()->angle;
		}
		
		// Angle PID
		s16 current_angle = get_pos()->angle;
		s16 angle_diff = angle_target - current_angle;
		
		if (angle_diff <= -1800) {
			angle_diff += 3600;
		} else if (angle_diff > 1800) {
			angle_diff -= 3600;
		}
		
		
		if (Abs(angle_diff) <= ANGLE_LOCK_THRESHOLD) {
			// Reached threshold
			//angle_target = get_pos()->angle;
			angle_lock_mv = 0;
		} else {
			s16 mv_tmp = angle_diff;
			// Limit
			if (angle_diff > Abs(ANGLE_LOCK_LIMIT)) {
				mv_tmp = Abs(ANGLE_LOCK_LIMIT);
			} else if (angle_diff < -Abs(ANGLE_LOCK_LIMIT)) {
				mv_tmp = -Abs(ANGLE_LOCK_LIMIT);
			}
			
			angle_lock_mv = ANGLE_LOCK_Kp * mv_tmp / 100;
		}
	} else {
		// Angle lock being ignored
		angle_target = get_pos()->angle;
		angle_lock_mv = 0;
	}
}

void angle_lock_ignore(u16 delay_ms)
{
	u32 ticks_tmp = get_full_ticks() + delay_ms;
	if (angle_lock_ignore_until_ticks < ticks_tmp) {
		angle_lock_ignore_until_ticks = ticks_tmp;
		angle_lock_mv = 0;
	}
}

/**
	* @brief Get the angle lock manipulated variable 
	*/
s16 angle_lock_get_mv(void)
{
	return angle_lock_mv;
}


s16 angle_lock_get_target(void)
{
	return angle_target;
}
