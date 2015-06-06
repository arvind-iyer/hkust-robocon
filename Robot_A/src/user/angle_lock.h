#ifndef	__ANGLE_LOCK_H
#define	__ANGLE_LOCK_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "gyro.h"
#include "approx_math.h"

#define	ANGLE_LOCK_THRESHOLD		30				// 3.0 degrees
#define	ANGLE_LOCK_LIMIT				150				// 20.0 degrees

#define	ANGLE_LOCK_Kp						180			// Scaled by 100


void angle_lock_init(void);
bool angle_lock_ignored(void);
void angle_lock_update(void);
void angle_lock_ignore(u16 delay_ms);
s16 angle_lock_get_mv(void);
s16 angle_lock_get_target(void);

#endif	/* __ANGLE_LOCK_H */
