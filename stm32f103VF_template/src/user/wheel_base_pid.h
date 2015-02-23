#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"

#define time_interval 10


typedef struct {
	s32 Kp_xy, Kp_w;
	s32 Ki_xy, Ki_w;
	s32 Kd_xy, Kd_w;
}PID_para;

typedef struct {
	s32 x;
	s32 y;
	s32 w;
}error;

typedef struct {
	error diff ;
	error derivative;
	error intergral;
}PID_error;

typedef struct {
	s32 x;
	s32 y;
	s32 w;
}PID_output_vel;


void wheel_base_pid_update(void);
void set_PID_val (PID_para para);

s32 get_vx(void);
s32 get_vy(void);
s32 get_vw(void);

#endif /* __WHEEL_BASE_PID */

