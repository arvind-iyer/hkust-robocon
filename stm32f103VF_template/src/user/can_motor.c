#include "can_motor.h"

void motor_set_vel(MOTOR_ID motor_id, s32 vel, CLOSE_LOOP_FLAG close_loop_flag)
{
	CAN_MESSAGE msg;
	msg.id = motor_id;
	
	msg.length = 6;
}
