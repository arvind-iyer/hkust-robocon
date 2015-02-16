#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"


void wheel_base_pid_loop(void);
void wheel_base_set_pid(PID pid);




#endif /* __WHEEL_BASE_PID */
