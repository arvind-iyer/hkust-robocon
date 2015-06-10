#include "angle_lock.h"
#include "gyro.h"

s16 angle_prev = 0;
s16 angle_target = 0;
u32 angle_lock_ignore_until_ticks = 0;

s16 angle_lock_mv = 0;

s32 angle_error_history[ANGLE_LOCK_Ki_SAMPLE] = {0};
u16 angle_lock_Ki_id = 0;

void angle_lock_init(void)
{
	angle_lock_mv = 0;
	angle_target = get_pos()->angle; 
	for (u8 i = 0; i < ANGLE_LOCK_Ki_SAMPLE; ++i) {
		angle_error_history[i] = 0;
		angle_lock_Ki_id = 0;
	}
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
		s16 angle_error = angle_target - current_angle;
		
		if (angle_error <= -1800) {
			angle_error += 3600;
		} else if (angle_error > 1800) {
			angle_error -= 3600;
		}
		
		
		if (Abs(angle_error) <= ANGLE_LOCK_THRESHOLD) {
			// Reached threshold
			//angle_target = get_pos()->angle;
			angle_lock_mv = 0;
		} else {
			
			s32 angle_error_sum = 0;
			// Kp limit
			if (angle_error > Abs(ANGLE_LOCK_Kp_LIMIT)) {
				angle_error = ANGLE_LOCK_Kp_LIMIT;
			} else if (angle_error < -Abs(ANGLE_LOCK_Kp_LIMIT)) {
				angle_error = -ANGLE_LOCK_Kp_LIMIT;
			}
			
			// Add id
			if (++angle_lock_Ki_id >= ANGLE_LOCK_Ki_SAMPLE) {
				angle_lock_Ki_id = 0;
			} 
			
			angle_error_history[angle_lock_Ki_id] = angle_error;
			
			for (u8 i = 0; i < ANGLE_LOCK_Ki_SAMPLE; ++i) {
				angle_error_sum += angle_error_history[i];
			}
			angle_lock_mv = ANGLE_LOCK_Kp * angle_error / 100 + ANGLE_LOCK_Ki * angle_error_sum / 100;
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
