#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"

#define time_interval 30


typedef struct {
	u32 Kp_xy, Kp_w;
	u32 Ki_xy, Ki_w;
	u32 Kd_xy, Kd_w;
}PID_para;

typedef struct {
	u32 x;
	u32 y;
	u32 w;
}error;

typedef struct {
	error diff ;
	error derivative;
	error intergral;
}PID_error;

typedef struct {
	u32 x;
	u32 y;
	u32 w;
}PID_output_vel;


void wheel_base_pid_update(void);
void set_PID_val (PID_para para);
void set_target_pos (POSITION pos);


#endif /* __WHEEL_BASE_PID */

