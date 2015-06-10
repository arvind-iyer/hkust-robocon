#ifndef	__ANGLE_LOCK_H
#define	__ANGLE_LOCK_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "gyro.h"
#include "approx_math.h"

#define	ANGLE_LOCK_THRESHOLD		10				// 1.0 degrees


#define	ANGLE_LOCK_Kp						220			// Scaled by 100
#define	ANGLE_LOCK_Kp_LIMIT			120			// 12.0 degrees

#define	ANGLE_LOCK_Ki						2			// Scaled by 100
#define	ANGLE_LOCK_Ki_SAMPLE		20			
	
void angle_lock_init(void);
bool angle_lock_ignored(void);
void angle_lock_update(void);
void angle_lock_ignore(u16 delay_ms);
s16 angle_lock_get_mv(void);
s16 angle_lock_get_target(void);

#endif	/* __ANGLE_LOCK_H */
