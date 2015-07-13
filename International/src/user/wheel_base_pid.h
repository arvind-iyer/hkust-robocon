#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"
#include "approx_math.h"

typedef	struct {
	s32 Kp, Ki, Kd;
} PID;

#define angle_para_prescalar 10000
#define DEFAULT_P 5000
#define DEFAULT_I 312
extern int angle_para[];

void wheel_base_pid_loop(void);
void wheel_base_set_pid(PID pid);
s32 pid_maintain_angle(void);
s32 delX(POSITION p1, POSITION p2);
s32 delY(POSITION p1, POSITION p2);
s32 delW(POSITION p1, POSITION p2);


#endif /* __WHEEL_BASE_PID */
