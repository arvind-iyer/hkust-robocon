#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"

// Protocol
#define	BLUETOOTH_WHEEL_BASE_AUTO_POS_ID			0x50
#define	BLUETOOTH_WHEEL_BASE_AUTO_START_ID		0x51
#define	BLUETOOTH_WHEEL_BASE_AUTO_STOP_ID			0x52

typedef	struct {
	s32 Kp, Ki, Kd;
} PID;

void wheel_base_pid_init(void);
void wheel_base_set_pid(PID pid);
POSITION wheel_base_get_target_pos(void);
void wheel_base_set_target_pos(POSITION pos);
void wheel_base_pid_on(void);
void wheel_base_pid_off(void);
u8 wheel_base_get_pid_flag(void);
void wheel_base_pid_loop(void);



#endif /* __WHEEL_BASE_PID */
