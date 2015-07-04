#include "stm32f10x.h"
#include "gyro.h"

#define	ANGLE_VARIANCE_SAMPLE			125


void angle_variance_update(void);
u32 get_angle_variance(void);
