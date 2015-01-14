#include "can_motor.h"

#define get_can_motor_id(motor_id)	(CAN_MOTOR_BASE + (u8)motor_id)
s32 can_motor_encoder_value[CAN_MOTOR_COUNT] = {0};

void can_motor_init(void)
{
	can_rx_add_filter(CAN_MOTOR_BASE, CAN_RX_MASK_DIGIT_0_F, 0);
}

/*** TX ***/
/**
	* @brief Set motor velocity (CAN)
	* @param motor_id (MOTOR_ID enum)
	* @param vel (vel of close_loop is not corresponded to open_loop)
	* @param close_loop_flag: true if close_loop should be applied
	* @retval None.
	*/
void motor_set_vel(MOTOR_ID motor_id, s32 vel, CLOSE_LOOP_FLAG close_loop_flag)
{
	CAN_MESSAGE msg;
	
	assert_param((u8)motor_id < CAN_MOTOR_COUNT);

	msg.id = get_can_motor_id(motor_id);
	msg.length = CAN_MOTOR_VEL_LENGTH;
	msg.data[0] = CAN_MOTOR_VEL_CMD;
	msg.data[1] = (u8)(one_to_n_bytes(vel, 0));
	msg.data[2] = (u8)(one_to_n_bytes(vel, 1));
	msg.data[3] = (u8)(one_to_n_bytes(vel, 2));
	msg.data[4] = (u8)(one_to_n_bytes(vel, 3));
	msg.data[5] = (u8)(close_loop_flag);
	
	can_tx_enqueue(msg);
}

/*** RX ***/
void can_motor_feedback_encoder(CanRxMsg msg)
{
	switch (msg.Data[0]) {
		case CAN_ENCODER_FEEDBACK:
			if (msg.DLC == CAN_ENCODER_FEEDBACK_LENGTH) {
				// Range check 
				if (msg.StdId >= CAN_MOTOR_BASE && msg.StdId < CAN_MOTOR_BASE + CAN_MOTOR_COUNT) {
					s32 feedback = n_bytes_to_one(&msg.Data[1], 4);
					can_motor_encoder_value[msg.StdId - CAN_MOTOR_BASE] = feedback;
				}
			}
		break;
	}
}

s32 get_encoder_value(MOTOR_ID motor_id)
{
	return can_motor_encoder_value[motor_id];
}


