#include "angle_variance.h"


static u16 angle_variance_id = 0;
static u16 angle_diff_history[ANGLE_VARIANCE_SAMPLE] = {0};

static u16 angle_prev = 0;

static u32 angle_variance = 0;

void angle_variance_update(void)
{
	if (!gyro_get_available()) {return;} 
	u16 angle = get_pos()->angle;
	u32 angle_variance_tmp = angle_variance;
	s16 angle_diff = angle - angle_prev;
	if (angle_diff < -1800) {
		angle_diff += 3600;
	} else if (angle_diff > 1800) {
		angle_diff -= 3600;
	}
	
	angle_variance_tmp -= angle_diff_history[angle_variance_id];
	angle_diff_history[angle_variance_id] = Abs(angle_diff);
	angle_variance_tmp += angle_diff_history[angle_variance_id];
	angle_variance = angle_variance_tmp;
	
	++angle_variance_id;
	if (angle_variance_id >= ANGLE_VARIANCE_SAMPLE) {
		angle_variance_id = 0;
	}
	
	angle_prev = angle;
}

u32 get_angle_variance(void)
{
	return angle_variance;
}

