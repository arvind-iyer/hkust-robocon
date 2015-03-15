#include "can_motor.h"

#define get_can_motor_id(motor_id)	(CAN_MOTOR_BASE + (u8)motor_id)
static s32 can_motor_encoder_value[CAN_MOTOR_COUNT] = {0};
static u16 feedback[16]={0};
 
/**
  * @brief The private (static) function for decoding CAN message
  * @param msg: the CAN msg for decoding
  */
static void can_motor_feedback_decoding(CanRxMsg msg) 
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

/**
  * @brief Motor (through CAN protocol) initialization 
  * @param None
  * @retval None 
  */
void can_motor_init(void)
{
	can_rx_add_filter(CAN_MOTOR_BASE, CAN_RX_MASK_DIGIT_0_F, can_motor_feedback_decoding);
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

/**
	* @brief Set motor position (CAN)
	* @param motor_id (MOTOR_ID enum)
	* @param vel (vel of close_loop is not corresponded to open_loop)
	* @param pos: The position need to move to relative to current encoder value.
	* @retval None.
	*/
void motor_set_pos(MOTOR_ID motor_id, u16 vel, s32 pos)
{
	CAN_MESSAGE msg;
	
	assert_param((u8)motor_id < CAN_MOTOR_COUNT);
	
	msg.id = get_can_motor_id(motor_id);
	msg.length = CAN_MOTOR_POS_LENGTH;
	msg.data[0] = CAN_MOTOR_POS_CMD;
	msg.data[1] = (u8)(one_to_n_bytes(vel, 0));
	msg.data[2] = (u8)(one_to_n_bytes(vel, 1));
	msg.data[3] = (u8)(one_to_n_bytes(pos, 0));
	msg.data[4] = (u8)(one_to_n_bytes(pos, 1));
	msg.data[5] = (u8)(one_to_n_bytes(pos, 2));
	msg.data[6] = (u8)(one_to_n_bytes(pos, 3));

	can_tx_enqueue(msg);
}

/**
	* @brief Set motor acceleration (CAN)
	* @param motor_id (MOTOR_ID enum)
	* @param accel: acceleration parameter of motor
	* @retval None.
	*/
void motor_set_acceleration(MOTOR_ID motor_id, u16 accel)
{
	CAN_MESSAGE msg;
	
	assert_param((u8)motor_id < CAN_MOTOR_COUNT);
	
	msg.id = get_can_motor_id(motor_id);
	msg.length = CAN_MOTOR_PARAMETER_LENGTH;
	msg.data[0] = CAN_MOTOR_PARAMETER_CMD;
	msg.data[1] = (u8)(one_to_n_bytes(accel, 0));
	msg.data[2] = (u8)(one_to_n_bytes(accel, 1));

	can_tx_enqueue(msg);
}

/**
	* @brief Lock and stop motor immediately (CAN)
	* @param motor_id (MOTOR_ID enum)
	* @retval None.
	*/
void motor_lock(MOTOR_ID motor_id)
{
	CAN_MESSAGE msg;
	
	assert_param((u8)motor_id < CAN_MOTOR_COUNT);
	
	msg.id = get_can_motor_id(motor_id);
	msg.length = CAN_MOTOR_LOCK_LENGTH;
	msg.data[0] = CAN_MOTOR_LOCK_CMD;

	can_tx_enqueue(msg);
}


/*** RX ***/
/**
  * @brief Get the motor encoder value (based on CAN rx result)
  * @param motor_id: The can motor ID
  * @retval The encoder value of the selected CAN motor
  */
s32 get_encoder_value(MOTOR_ID motor_id)
{
	return can_motor_encoder_value[motor_id];
}



/// added for light sensor

void can_sensor_init(void){
	can_rx_add_filter(0x000, 0x000 , can_sensor_feedback_decoding);
}

static void can_sensor_feedback_decoding(CanRxMsg msg){
	switch (msg.Data[0]) {
		case LIGHTSENSOR_ENABLE:
			if (msg.DLC == 7) {
				// Range check 
				feedback[0] = msg.Data [1];
				feedback[1] = msg.Data [2];
				feedback[2] = msg.Data [3];
				feedback[3] = msg.Data [4];
				feedback[4] = msg.Data [5];
				feedback[5] = msg.Data [6];
			}
		break;
		case LIGHTSENSOR_DISABLE:
			if (msg.DLC == 7) {
				// Range check 
				feedback[6] = (u16)n_bytes_to_one(&msg.Data[1], 2);
				feedback[7] = (u16) n_bytes_to_one(&msg.Data[3], 2);
				feedback[8] = (u16)n_bytes_to_one(&msg.Data[5], 2);
			}
		break;
		case LIGHT_SENSOR_2:
			if (msg.DLC == 7) {
				// Range check 
				feedback[9] = (u16)n_bytes_to_one(&msg.Data[1], 2);
				feedback[10] = (u16)n_bytes_to_one(&msg.Data[3], 2);
				feedback[11] = (u16)n_bytes_to_one(&msg.Data[5], 2);
			}
		break;
		case LIGHT_SENSOR_3:
			if (msg.DLC == 4) {
				// Range check 
				feedback[12] = (u16)n_bytes_to_one(&msg.Data[1], 2);
				feedback[13] = msg.Data [3];
				
			}
		break;
	}
}

	

u8 get_sensor_feedback_0 (void){
	return feedback[0] ;
}
u8 get_sensor_feedback_1 (void){
	return feedback[1] ;
}
u8 get_sensor_feedback_2 (void){
	return feedback[2] ;
}

u8 get_sensor_feedback_3 (void){
	return feedback[3] ;
}

u8 get_sensor_feedback_4 (void){
	return feedback[4] ;
}
u8 get_sensor_feedback_5 (void){
	return feedback[5] ;
}
u8 get_sensor_feedback_6 (void){
	return feedback[6] ;
}
u8 get_sensor_feedback_7 (void){
	return feedback[7] ;
}
u8 get_sensor_feedback_8 (void){
	return feedback[8] ;
}
u8 get_sensor_feedback_9 (void){
	return feedback[9] ;
}
u8 get_sensor_feedback_10 (void){
	return feedback[10] ;
}
u8 get_sensor_feedback_11 (void){
	return feedback[11] ;
}
u8 get_sensor_feedback_12 (void){
	return feedback[12] ;
}
u8 get_sensor_feedback_13 (void){
	return feedback[13] ;
}
u8 get_sensor_feedback_14 (void){
	return feedback[14] ;
}
u8 get_sensor_feedback_15 (void){
	return feedback[15] ;
}






