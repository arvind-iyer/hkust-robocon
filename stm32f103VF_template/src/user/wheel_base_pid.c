#include "wheel_base_pid.h"

static u8 wheel_base_init_flag = 0;
static u8 wheel_base_pid_flag = 0;
static POSITION target_pos = {0, 0, 0};
static PID wheel_base_pid = {0, 0, 0};

static void wheel_base_auto_bluetooth_decode(u8 id, u8 length, u8* data)
{
	switch (id) {
		case BLUETOOTH_WHEEL_BASE_AUTO_POS_ID:
			if (length == 6) {
				target_pos.x = ((data[0] << 8) & 0xFF) | (data[1] & 0xFF);
				target_pos.y = ((data[2] << 8) & 0xFF) | (data[3] & 0xFF);
				target_pos.angle = ((data[4] << 8) & 0xFF) | (data[5] & 0xFF);
				
			}
		break;
		
		case BLUETOOTH_WHEEL_BASE_AUTO_START_ID:
			if (length == 0) {
				wheel_base_pid_on();
			}
		break;

			
		case BLUETOOTH_WHEEL_BASE_AUTO_STOP_ID:
			wheel_base_pid_off();
		break;
	}
}

void wheel_base_pid_init(void)
{
	bluetooth_rx_add_filter(BLUETOOTH_WHEEL_BASE_AUTO_POS_ID, 0xF0, wheel_base_auto_bluetooth_decode);
	wheel_base_init_flag = 1;
}
/**
	* @brief Set PID
	* @param PID to be set
	* @retval None.
	*/
void wheel_base_set_pid(PID pid)
{
	wheel_base_pid.Kp = pid.Kp;
	wheel_base_pid.Ki = pid.Ki;
	wheel_base_pid.Kd = pid.Kd;
}

POSITION wheel_base_get_target_pos(void)
{
	return target_pos;
}

void wheel_base_set_target_pos(POSITION pos) {
	target_pos = pos;
}

void wheel_base_pid_on(void)
{
	wheel_base_pid_flag = 1;
}

void wheel_base_pid_off(void)
{
	wheel_base_pid_flag = 0;
}

u8 wheel_base_get_pid_flag(void)
{
	return wheel_base_pid_flag;
}

void wheel_base_pid_loop(void)
{
	if (wheel_base_init_flag) {
		/** TODO: Code the auto PID **/
		/** Use wheel_base_set_vel(x,y,w) to control wheel base motors */
	}
}

