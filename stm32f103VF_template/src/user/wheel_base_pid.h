#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"

void wheel_base_pid_init(void);
void wheel_base_pid_update(void);

// functions for xbox
void set_starting_pos(void);
void set_serving_pos(void);
void set_returning_pos(void);
void set_after_serve_pos(void);

// getters
int get_x_speed(void);
int get_y_speed(void);
int get_t_speed(void);
int get_prop(void);
int get_int(void);
int get_der(void);
u8 get_pid_stat(void);

#endif /* __WHEEL_BASE_PID */
