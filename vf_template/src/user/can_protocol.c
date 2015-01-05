/**********************************************************************
*				HKUST ROBOCON 2014 CAN PROTOCOL						
*		
*Author: Joey Bi
*Date: JAN, 2014		
*Description: This file defines the CAN communication protocol
			  between the main control and the sub-systems.
				!! Developed from 2013 CAN PROTOCOL by ZHANG LIANG GUANG !!
*Updates: 
*			The New Sensor Board CAN Complete     --    JAN 5, 2014
*			New CAN Protocol to transmit SENSOR signal -- JAN 6, 2014

***********************************************************************/

#include "can_protocol.h"

//#include "tft_160x128.h"

/* Public variables ------------------------------------------*/
s32 Encoder_Count[NUM_OF_MOTORS] = {0};			//encoder feedback from the motor controller
s32 Motor_Valid[NUM_OF_MOTORS] = {0};			//encoder feedback from the motor controller
s16	Robot_Coordinate_X = 0;						//position feedback from the position board
s16 Robot_Coordinate_Y = 0;
s16 Robot_Angle = 0;
s16 Robot_Velocity_X = 0;						//real velocity feedback from the position board
s16 Robot_Velocity_Y = 0;
s16 Robot_Angular_Velocity = 0;
u32 IO_Input_Data = 0;
u32 IO_Output_Data = 0;
u8 	Sensor_State[14];
u8  Cylinder_State[16];  
s32 CAN_motor_vel = 0;
u8  CAN_motor_flag = 0;
u16 CAN_motor_vel_mag = 0;
s32 CAN_motor_pos = 0;	
	
/* Private variables -----------------------------------------*/
u16 sensor_state;
u32 button_state;
CanTxMsg Tx_Queue[TX_QUEUE_LENGTH];
u8 Tx_QueueHead = 0;
u8 Tx_QueueTail = 0;
u8 Tx_QueueCounter = 0;
CanRxMsg Rx_Queue[RX_QUEUE_LENGTH];
u8 Rx_QueueHead = 0;
u8 Rx_QueueTail = 0;
u8 Rx_QueueCounter = 0;
u8 sensor_bar_result1;
u8 sensor_bar_result2;
u16 laser_state1;
u16 laser_state2;
u16 laser_state3;
u16 laser_state4;
u8 laser_Q1;
u8 laser_Q2;
u8 laser_Q3;
u8 laser_Q4;

u8 Light_Count;

vu16 ADC_ConvertedValue[4];

//Ports and Pins of the sensor board
#ifdef SENSOR_BOARD
//extern GPIO_TypeDef* Sensor_Board_SIG_PORT_Array[14];
//extern const uint16_t Sensor_Board_SIG_Pin_Array[14];
//extern GPIO_TypeDef* Sensor_Board_PNEU_PORT_Array[16];
//extern const uint16_t Sensor_Board_PNEU_Pin_Array[16];
#endif
#ifdef BUTTON_BOARD
extern const uint16_t Button_Board_Pin_Array[32];
extern GPIO_TypeDef* Button_Board_PORT_Array[32];
#endif

/*****************************************************************
*				Data decomposition and reconstruction				
*		
*Author: MA Fangchang, Modifed by ZHANG Linguang
*Year: 2012			
*Description: These functions decomposite and reconstruct the s16
			  and s32 data
******************************************************************/

s32 Four_Bytes_Reconstruction(const uint8_t* buffer)
{
	s32 data=0;
	//也是低位到高位
	data |= buffer[0]; 
	data |= buffer[1] << 8;
	data |= buffer[2] << 16;
	data |= buffer[3] << 24;
	return data;
}

s16 Two_Bytes_Reconstruction(const uint8_t* buffer)
{																												
	s16 data=0;
	data |= buffer[0]; 
	data |= buffer[1] << 8;
	return data;
}

u8 Four_Bytes_Decomposition(const s32 data, const u8 index)
{
	switch (index)
	{
		//低位到高位
		case 0: return (data & 0x000000ff);
		case 1: return (data >> 8) & 0x000000ff;
		case 2: return (data >> 16) & 0x000000ff;
		case 3: return (data >> 24) & 0x000000ff;
		default: break;
	}
	return 0;
}

u8 Two_Bytes_Decomposition(const u16 data, const u8 index)
{
	switch (index)
	{
		case 0: return (data & 0x00ff);
		case 1: return (data >> 8) & 0x00ff;
		default: break;
	}
	return 0;
}


/* General Command Encoding ------------------------------------------------------*/
CanTxMsg General_Encoding(u32 Device_ID, Data_Field Cmd_Data)
{
	u8 i;
	CanTxMsg TxMessage; 
	TxMessage.StdId = Device_ID;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = Cmd_Data.Data_Length;
	for(i = 0; i < Cmd_Data.Data_Length; i++)
	{
		TxMessage.Data[i] = Cmd_Data.Data[i];		
	}
	return TxMessage;
}

/*=========================MOTOR CONTROL=======================*/

CanTxMsg Motor_Velocity_Encoding(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = MOTOR_VEL_CMD_LENGTH;
	Raw_Data.Data[0] = MOTOR_VEL_CMD;								//indicates the type of cmd
	Raw_Data.Data[1] = Four_Bytes_Decomposition((u32)(vel), 0);		//decomposite the velocity
	Raw_Data.Data[2] = Four_Bytes_Decomposition((u32)(vel), 1);		
	Raw_Data.Data[3] = Four_Bytes_Decomposition((u32)(vel), 2);		
	Raw_Data.Data[4] = Four_Bytes_Decomposition((u32)(vel), 3);	
	Raw_Data.Data[5] = Closed_Loop_Flag;							//indicates whether the control is 
																	//closed-loop or opened-loop
	return General_Encoding(Motor_ID, Raw_Data);					//data field is ready, encode the CAN data frame	
}

CanTxMsg Motor_Position_Encoding(u32 Motor_ID, u16 vel, s32 pos)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = MOTOR_POS_CMD_LENGTH;
	Raw_Data.Data[0] = MOTOR_POS_CMD;								//indicates the type of cmd
	Raw_Data.Data[1] = Two_Bytes_Decomposition(vel, 0);				//decomposite the velocity
	Raw_Data.Data[2] = Two_Bytes_Decomposition(vel, 1); 
	Raw_Data.Data[3] = Four_Bytes_Decomposition(pos, 0);			//decomposite the position
	Raw_Data.Data[4] = Four_Bytes_Decomposition(pos, 1);
	Raw_Data.Data[5] = Four_Bytes_Decomposition(pos, 2);
	Raw_Data.Data[6] = Four_Bytes_Decomposition(pos, 3);
	return General_Encoding(Motor_ID, Raw_Data);					//data field is ready, encode the CAN data frame	
}

CanTxMsg Motor_Parameter_Encoding(u32 Motor_ID, u16 accel)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = MOTOR_PARAMETER_CMD_LENGTH;
	Raw_Data.Data[0] = MOTOR_PARAMETER_CMD;							//indicates the type of cmd
	Raw_Data.Data[1] = Two_Bytes_Decomposition(accel, 0);	
	Raw_Data.Data[2] = Two_Bytes_Decomposition(accel, 1);
	return General_Encoding(Motor_ID, Raw_Data);
}

CanTxMsg Motor_Lock_Encoding(u32 Motor_ID)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = MOTOR_LOCK_CMD_LENGTH;
	Raw_Data.Data[0] = MOTOR_LOCK_CMD;								//indicates the type of cmd
	return General_Encoding(Motor_ID, Raw_Data);
}

/**********************************************************************
*				Motor Control Feedback Encoding
*Description: These functions encode the feedback that the motor control 
*				sends to the main control
*Variables explanation:
***********************************************************************/

CanTxMsg Encoder_Feedback_Encoding(u32 Motor_ID, s32 Encoder_Val)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = ENCODER_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = ENCODER_FEEDBACK;
	Raw_Data.Data[1] = Four_Bytes_Decomposition(Encoder_Val, 0);
	Raw_Data.Data[2] = Four_Bytes_Decomposition(Encoder_Val, 1);
	Raw_Data.Data[3] = Four_Bytes_Decomposition(Encoder_Val, 2);
	Raw_Data.Data[4] = Four_Bytes_Decomposition(Encoder_Val, 3);
	return General_Encoding(Motor_ID, Raw_Data);		
}

CanTxMsg Data1_Feedback_Encoding(u32 Motor_ID, s32 Encoder_Val)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = DATA1_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = DATA1_FEEDBACK;
	Raw_Data.Data[1] = Four_Bytes_Decomposition(Encoder_Val, 0);
	Raw_Data.Data[2] = Four_Bytes_Decomposition(Encoder_Val, 1);
	Raw_Data.Data[3] = Four_Bytes_Decomposition(Encoder_Val, 2);
	Raw_Data.Data[4] = Four_Bytes_Decomposition(Encoder_Val, 3);
	return General_Encoding(Motor_ID, Raw_Data);		
}

CanTxMsg Data2_Feedback_Encoding(u32 Motor_ID, s32 Encoder_Val)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = DATA2_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = DATA2_FEEDBACK;
	Raw_Data.Data[1] = Four_Bytes_Decomposition(Encoder_Val, 0);
	Raw_Data.Data[2] = Four_Bytes_Decomposition(Encoder_Val, 1);
	Raw_Data.Data[3] = Four_Bytes_Decomposition(Encoder_Val, 2);
	Raw_Data.Data[4] = Four_Bytes_Decomposition(Encoder_Val, 3);
	return General_Encoding(Motor_ID, Raw_Data);		
}

CanTxMsg Data3_Feedback_Encoding(u32 Motor_ID, s32 Encoder_Val)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = DATA3_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = DATA3_FEEDBACK;
	Raw_Data.Data[1] = Four_Bytes_Decomposition(Encoder_Val, 0);
	Raw_Data.Data[2] = Four_Bytes_Decomposition(Encoder_Val, 1);
	Raw_Data.Data[3] = Four_Bytes_Decomposition(Encoder_Val, 2);
	Raw_Data.Data[4] = Four_Bytes_Decomposition(Encoder_Val, 3);
	return General_Encoding(Motor_ID, Raw_Data);		
}

CanTxMsg Data4_Feedback_Encoding(u32 Motor_ID, s32 Encoder_Val)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = DATA4_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = DATA4_FEEDBACK;
	Raw_Data.Data[1] = Four_Bytes_Decomposition(Encoder_Val, 0);
	Raw_Data.Data[2] = Four_Bytes_Decomposition(Encoder_Val, 1);
	Raw_Data.Data[3] = Four_Bytes_Decomposition(Encoder_Val, 2);
	Raw_Data.Data[4] = Four_Bytes_Decomposition(Encoder_Val, 3);
	return General_Encoding(Motor_ID, Raw_Data);		
}


void Motor_Cmd_Decoding(CanRxMsg RxMessage)
{
	u8 tmp1[4] = {0};
	u8 tmp2[2] = {0};
	u8 i = 0;
	//For velocity CMD
	s32 recombined_vel;
	u8 closed_loop_flag;
	//For position CMD
	u16 recombined_vel_mag;
	s32 recombined_pos;
	//For acceleration CMD:
	u16 accel_val;

	switch(RxMessage.Data[0])
	{
		case MOTOR_VEL_CMD:
			for(i = 0; i < 4; i++)
				tmp1[i] = RxMessage.Data[i+1];
			recombined_vel = Four_Bytes_Reconstruction(tmp1);
			closed_loop_flag = RxMessage.Data[5];
			//CALL ZHAOZHETUO'S FUNCTION
			//SET_VEL(VEL);

			CAN_motor_vel = recombined_vel;
			CAN_motor_flag = closed_loop_flag;
			break;
				
		case MOTOR_POS_CMD:
			for(i = 0; i < 2; i++)
				tmp2[i] = RxMessage.Data[i+1];
			for(i = 0; i < 4; i++)
				tmp1[i] = RxMessage.Data[i+3];
			recombined_vel_mag = (u16)(Two_Bytes_Reconstruction(tmp2));
			recombined_pos = Four_Bytes_Reconstruction(tmp1);
			//CALL ZHAOZHETUO'S FUNCTION
			//SET_POS(VEL,POS);	
			break;

		case MOTOR_LOCK_CMD:
			//motor_lock();
			break;

		case MOTOR_PARAMETER_CMD:
			for(i = 0; i < 4; i++)
				tmp1[i] = RxMessage.Data[i+1];
			accel_val = Four_Bytes_Reconstruction(tmp1);
			//CALL ZHAOZHETUO'S FUNCTION
			//set_acceleration(accel_val);
			break;

		default:
			break;
	}
}	
/**********************************************************************
*				Motor Control Feedback Decoding
*Description: These functions decode the feedback that the motor control 
*				send to the main control
*Variables explanation:
***********************************************************************/

void Motor_Feedback_Decoding(CanRxMsg RxMessage)
{
	u8 i;
	u8 tmp[4];
	s32 feedback = 0;
	if(RxMessage.Data[0] ==  ENCODER_FEEDBACK){
		for(i = 0; i < 4; i++)
		{
			tmp[i] = RxMessage.Data[i+1];
		}
		feedback = Four_Bytes_Reconstruction(tmp);
	}
	if(RxMessage.Data[0] ==  ENCODER_FEEDBACK)
	{
		Encoder_Count[RxMessage.StdId - MOTOR_ID_OFFSET] = feedback;
	}
	else if(RxMessage.Data[0] ==  MOTOR_CAL_FEEDBACK)
	{
		Motor_Valid[RxMessage.StdId - MOTOR_ID_OFFSET] = 1;			
	}
}

/*=====================Positioning System ========================*/

CanTxMsg Gyro_Stop_Calibration_Encoding()
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = GYRO_STOP_CAL_CMD_LENGTH;	
	Raw_Data.Data[0] = GYRO_STOP_CAL_CMD;
  return General_Encoding(POS_SYSTEM, Raw_Data);
}

CanTxMsg Gyro_Set_Pos_Encoding(s16 Pos_X, s16 Pos_Y, s16 Angle)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = GYRO_SET_CMD_LENGTH;	
	Raw_Data.Data[0] = GYRO_SET_CMD;
	Raw_Data.Data[1] = Two_Bytes_Decomposition((u16)(Pos_X), 0);
	Raw_Data.Data[2] = Two_Bytes_Decomposition((u16)(Pos_X), 1);
	Raw_Data.Data[3] = Two_Bytes_Decomposition((u16)(Pos_Y), 0);
	Raw_Data.Data[4] = Two_Bytes_Decomposition((u16)(Pos_Y), 1);
	Raw_Data.Data[5] = Two_Bytes_Decomposition((u16)(Angle), 0);
	Raw_Data.Data[6] = Two_Bytes_Decomposition((u16)(Angle), 1);
   	return General_Encoding(POS_SYSTEM, Raw_Data);
}

void Gyro_Cmd_Decoding(CanRxMsg RxMessage)
{
	u8 i;
	u8 tmp1[2];
	u8 tmp2[2];
	u8 tmp3[2];	
	s16 new_x;
	s16 new_y;
	s16 new_angle;
	switch(RxMessage.Data[0])
	{
		case GYRO_STOP_CAL_CMD:
			//CALL XIAOJIEJIE'S FUNCTION
			//GYRO_CAL();
			break;
		case GYRO_SET_CMD:
			//s32 feedback = 0;
			for(i = 0; i < 2; i++)
				tmp1[i] = RxMessage.Data[i+1];
			for(i = 0; i < 2; i++)
				tmp2[i] = RxMessage.Data[i+3];
			for(i = 0; i < 2; i++)
				tmp3[i] = RxMessage.Data[i+5];
			new_x = Two_Bytes_Reconstruction(tmp1);							
			new_y = Two_Bytes_Reconstruction(tmp2);					  
			new_angle = Two_Bytes_Reconstruction(tmp3);		
			//CALL XIAOJIEJIE'S FUNCTION
			//GYRO_SET_POS(new_x, new_y, new_angle);
			break;
		default:
			break;
	}
}

/**********************************************************************
*				Position System Feedback Decoding
*Description: These functions decode the feedback that the motor control 
*				send to the main control
*Variables explanation:
***********************************************************************/

void Gyroscope_Feedback_Decoding(CanRxMsg RxMessage)
{
	u8 i;
	u8 tmp1[2];
	u8 tmp2[2];
	u8 tmp3[2];
	//s32 feedback = 0;
	for(i = 0; i < 2; i++)
		tmp1[i] = RxMessage.Data[i+1];
	for(i = 0; i < 2; i++)
		tmp2[i] = RxMessage.Data[i+3];
	for(i = 0; i < 2; i++)
		tmp3[i] = RxMessage.Data[i+5];
	if(RxMessage.Data[0] ==  POS_FEEDBACK)				//if the data frame contains the position
	{
		Robot_Coordinate_X = Two_Bytes_Reconstruction(tmp1);							
		Robot_Coordinate_Y = Two_Bytes_Reconstruction(tmp2);					  
		Robot_Angle = Two_Bytes_Reconstruction(tmp3);		
	}
	else if(RxMessage.Data[0] ==  VEL_FEEDBACK)			//if the data frame contains the velocity
	{
		Robot_Velocity_X = Two_Bytes_Reconstruction(tmp1);					
		Robot_Velocity_Y = Two_Bytes_Reconstruction(tmp2);
		Robot_Angular_Velocity = Two_Bytes_Reconstruction(tmp3);	 	
	}
	else if(RxMessage.Data[0] ==  GYRO_CAL_FEEDBACK){
		//Gyro_Cal_Flag = RxMessage.Data[1];
		//Robot_Coordinate_X++;
	}
}

void Gyroscope_Feedback_Decoding_NEW(CanRxMsg RxMessage)
{
	u8 i;
	u8 tmp1[2];
	//s32 feedback = 0;
	for(i = 0; i < 2; i++)
		tmp1[i] = RxMessage.Data[i];				  
	Robot_Angle = Two_Bytes_Reconstruction(tmp1);		
	
}

void Gyroscope_Feedback_Decoding_NEW_2(CanRxMsg RxMessage)
{
	u8 i;
	u8 tmp1[2];
	u8 tmp2[2];
	u8 tmp3[2];
	//s32 feedback = 0;
	for(i = 0; i < 2; i++)
		tmp1[i] = RxMessage.Data[i];
	for(i = 0; i < 2; i++)
		tmp2[i] = RxMessage.Data[i+2];
	for(i = 0; i < 2; i++)
		tmp3[i] = RxMessage.Data[i+4];

		Robot_Coordinate_X = Two_Bytes_Reconstruction(tmp1);							
		Robot_Coordinate_Y = Two_Bytes_Reconstruction(tmp2);					  
		Robot_Angle = Two_Bytes_Reconstruction(tmp3);		
	
}

/**********************************************************************
*				Position System Feedback Encoding
*Description: These functions encode the feedback that the position
*				system sends to the main control
*Variables explanation:
***********************************************************************/

CanTxMsg Pos_Feedback_Encoding(s16 Pos_X, s16 Pos_Y, s16 Angle)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = POS_FEEDBACK_LENGTH;	
	Raw_Data.Data[0] = POS_FEEDBACK;
	Raw_Data.Data[1] = Two_Bytes_Decomposition((u16)(Pos_X), 0);
	Raw_Data.Data[2] = Two_Bytes_Decomposition((u16)(Pos_X), 1);
	Raw_Data.Data[3] = Two_Bytes_Decomposition((u16)(Pos_Y), 0);
	Raw_Data.Data[4] = Two_Bytes_Decomposition((u16)(Pos_Y), 1);
	Raw_Data.Data[5] = Two_Bytes_Decomposition((u16)(Angle), 0);
	Raw_Data.Data[6] = Two_Bytes_Decomposition((u16)(Angle), 1);
  return General_Encoding(POS_SYSTEM, Raw_Data);
}


CanTxMsg Pos_Feedback_Encoding_NEW(u16 Angle)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = 2;	
	Raw_Data.Data[0] = Two_Bytes_Decomposition((u16)(Angle), 0);
	Raw_Data.Data[1] = Two_Bytes_Decomposition((u16)(Angle), 1);
  return General_Encoding(NEW_POS_SYSTEM, Raw_Data);
}

CanTxMsg Pos_Feedback_Encoding_NEW_2(u16 x, s16 y, u16 Angle)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = 6;	
	Raw_Data.Data[0] = Two_Bytes_Decomposition((u16)x, 0);
	Raw_Data.Data[1] = Two_Bytes_Decomposition((u16)x, 1);	
	Raw_Data.Data[2] = Two_Bytes_Decomposition((u16)y, 0);
	Raw_Data.Data[3] = Two_Bytes_Decomposition((u16)y, 1);	
	Raw_Data.Data[4] = Two_Bytes_Decomposition((u16)(Angle), 0);
	Raw_Data.Data[5] = Two_Bytes_Decomposition((u16)(Angle), 1);
  return General_Encoding(NEW_POS_SYSTEM_2, Raw_Data);
}

CanTxMsg Vel_Feedback_Encoding(s16 Vel_X, s16 Vel_Y, s16 Omega)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = VEL_FEEDBACK_LENGTH;	
	Raw_Data.Data[0] = VEL_FEEDBACK;
	Raw_Data.Data[1] = Two_Bytes_Decomposition((u16)(Vel_X), 0);
	Raw_Data.Data[2] = Two_Bytes_Decomposition((u16)(Vel_X), 1);
	Raw_Data.Data[3] = Two_Bytes_Decomposition((u16)(Vel_Y), 0);
	Raw_Data.Data[4] = Two_Bytes_Decomposition((u16)(Vel_Y), 1);
	Raw_Data.Data[5] = Two_Bytes_Decomposition((u16)(Omega), 0);
	Raw_Data.Data[6] = Two_Bytes_Decomposition((u16)(Omega), 1);
	return General_Encoding(POS_SYSTEM, Raw_Data);
}


void Gyro_Send_Pos(u16 Pos_Angle)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Pos_Feedback_Encoding_NEW(Pos_Angle);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void Gyro_Send_Pos_2(s16 x,s16 y,u16 Pos_Angle)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Pos_Feedback_Encoding_NEW_2(x,y,Pos_Angle);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

//####################################################################//
//----------------------------SENSOR BOARD----------------------------//
//####################################################################//
#ifdef SENSOR_BOARD
CanTxMsg Sensor_Board_Sensor_Encoding(){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = SEN_DATA_LENGTH;
	Raw_Data.Data[0] = SEN_DATA;
	Raw_Data.Data[1] = sensor_state & 0xFF;
	Raw_Data.Data[2] = (sensor_state>>8) & 0xFF;
  
	return General_Encoding(SEN_BOARD,Raw_Data);
}

CanTxMsg Sensor_Board_Laser_Encoding1(){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = LASER_DATA1_LENGTH;
	Raw_Data.Data[0] = LASER_DATA1;
	Raw_Data.Data[1] = laser_state1 & 0xFF;
	Raw_Data.Data[2] = (laser_state1>>8) & 0xFF;
	
	Raw_Data.Data[3] = laser_state2 & 0xFF;
	Raw_Data.Data[4] = (laser_state2>>8) & 0xFF;
	
	Raw_Data.Data[5] = laser_Q1<<1|laser_Q2;
	return General_Encoding(SEN_BOARD,Raw_Data);
}

CanTxMsg Sensor_Board_Laser_Encoding2(){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = LASER_DATA2_LENGTH;
	Raw_Data.Data[0] = LASER_DATA2;
	Raw_Data.Data[1] = laser_state3 & 0xFF;
	Raw_Data.Data[2] = (laser_state3>>8) & 0xFF;
	
	Raw_Data.Data[3] = laser_state4 & 0xFF;
	Raw_Data.Data[4] = (laser_state4>>8) & 0xFF;
	
	Raw_Data.Data[5] = laser_Q3<<1|laser_Q4;
	return General_Encoding(SEN_BOARD,Raw_Data);
}

void LightCodeAnalysis(u8 code){
	int i=1;
	enableLight=1;
	Light_Count = code;
}

void Sensor_Board_Cmd_Decoding(CanRxMsg RxMessage){
	
	switch(RxMessage.Data[0])
	{
		case SEN_DATA:
			// No action for now
			break;
		case CYL_DATA:
			GPIO_WriteBit(Sensor_Board_PNEU_PORT_Array[RxMessage.Data[1]],Sensor_Board_PNEU_Pin_Array[RxMessage.Data[1]],RxMessage.Data[2]);
			//tft_prints(0,1,"%d:%d",RxMessage.Data[1],RxMessage.Data[2]);
			break;
		case LIGHT_DATA:
			LightCodeAnalysis(RxMessage.Data[1]);
		
		default:
			break;
	}
}
#endif

CanTxMsg Sensor_Board_Cylinder_Encoding(u8 cylinderId, u8 Status){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = CYL_DATA_LENGTH;
	Raw_Data.Data[0] = CYL_DATA;											
	Raw_Data.Data[1] = cylinderId;										// 0~F
	Raw_Data.Data[2] = Status;												// Status = CYL_OPEN or CYLCLOSE
	return General_Encoding(SEN_BOARD,Raw_Data);
}

CanTxMsg Sensor_Board_Light_Encoding(u8 code){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = LIGHT_DATA_LENGTH;
	Raw_Data.Data[0] = LIGHT_DATA;											
	Raw_Data.Data[1] = code;
	return General_Encoding(SEN_BOARD,Raw_Data);
}

void Sensor_Board_Feedback_Decoding(CanRxMsg RxMessage){
	u8 i;
	switch(RxMessage.Data[0])
	{
		case SEN_DATA:
			sensor_state=RxMessage.Data[2]<<8|RxMessage.Data[1];
			break;
		case CYL_DATA:
			Cylinder_State[RxMessage.Data[1]]=RxMessage.Data[2];
			break;
		case LASER_DATA1:
			laser_state1=RxMessage.Data[2]<<8|RxMessage.Data[1];
			laser_state2=RxMessage.Data[4]<<8|RxMessage.Data[3];
			laser_Q1=RxMessage.Data[5]>>1&0xff;
		  laser_Q2=RxMessage.Data[5]&0xff;;
			
			break;
		case LASER_DATA2:
			laser_state3=RxMessage.Data[2]<<8|RxMessage.Data[1];
			laser_state4=RxMessage.Data[4]<<8|RxMessage.Data[3];
			laser_Q3=RxMessage.Data[5]>>1&0xff;
		  laser_Q4=RxMessage.Data[5]&0xff;;
			break;
		default:
			break;
	}
}



//####################################################################//
//----------------------------SENSOR BAR----------------------------//
//####################################################################//
#ifdef SENSOR_BAR
CanTxMsg Sensor_Bar_Encoding(){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = 1;
	Raw_Data.Data[0] = final_result;
	return General_Encoding(SEN_BAR,Raw_Data);
}
#endif

void Sensor_Bar1_Feedback_Decoding(CanRxMsg RxMessage){
	sensor_bar_result1=RxMessage.Data[0];
}

void Sensor_Bar2_Feedback_Decoding(CanRxMsg RxMessage){
	sensor_bar_result2=RxMessage.Data[0];
}

//####################################################################//
//---------------------------BUTTON BOARD-----------------------------//
//####################################################################//

CanTxMsg Button_Board_Sensor_Encoding(){
	Data_Field Raw_Data;
	Raw_Data.Data_Length = BTN_DATA_LENGTH;
	Raw_Data.Data[0] = button_state>>24&0xFF;
	Raw_Data.Data[1] = button_state>>16&0xFF;
	Raw_Data.Data[2] = button_state>>8&0xFF;
	Raw_Data.Data[3] = button_state&0xFF;
	return General_Encoding(BTN_BOARD,Raw_Data);
}

void Button_Board_Feedback_Decoding(CanRxMsg RxMessage){
	button_state = RxMessage.Data[0]<<24|RxMessage.Data[1]<<16|RxMessage.Data[2]<<8|RxMessage.Data[3];
}




/*==================== Controller ===================*/
void Controller_Feedback_Decoding(CanRxMsg RxMessage){
	//controller_data = RxMessage.Data[0]<<16 | RxMessage.Data[1]<<8 | RxMessage.Data[2];
}


//####################################################################//
//-----------------------CAN MESSAGE PROCESSING-----------------------//
//####################################################################//

/* CAN Tx message add to queue ------------------------------------------*/
void CAN_Tx_addToQueue(CanTxMsg TxMessage)
{
	Tx_Queue[Tx_QueueTail] = TxMessage;
	Tx_QueueTail++;
	if(Tx_QueueTail == TX_QUEUE_LENGTH)
		Tx_QueueTail = 0;
	Tx_QueueCounter++;
}

/* CAN Tx message dequeue and transmission -------------------------------*/
u8 CAN_Tx_dequeue(void)
{
	CanTxMsg TxMsg;
	u8 Tx_MailBox = 0;
	u16 retry = RETRY_TIMEOUT;
	if(Tx_QueueCounter != 0)
	{
		TxMsg = Tx_Queue[Tx_QueueHead];
		Tx_QueueHead++;
		if(Tx_QueueHead == TX_QUEUE_LENGTH)
			Tx_QueueHead = 0;
		Tx_QueueCounter--;
		Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
		while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);	//wait for the transmission to be finished
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;			
}


void CAN_Rx_addToQueue(CanRxMsg RxMessage)
{										   	
	Rx_Queue[Rx_QueueTail] = RxMessage;
	Rx_QueueTail++;
	if(Rx_QueueTail == RX_QUEUE_LENGTH)
		Rx_QueueTail = 0;
	Rx_QueueCounter++;
}

/* CAN Rx message processing ---------------------------------------------*/
void CAN_Rx_Processing(CanRxMsg RxMessage)
{		
	//GPIO_WriteBit(Sensor_Board_PNEU_PORT_Array[14],Sensor_Board_PNEU_Pin_Array[14],1);
	#ifdef MAIN_CONTROL
	if(RxMessage.StdId == 0x0A0)											//ID of the gyro
	{
		Gyroscope_Feedback_Decoding(RxMessage);
	}
	else if(RxMessage.StdId == 0x0A1)											//ID of the new gyro
	{
		Gyroscope_Feedback_Decoding_NEW(RxMessage);
	}
	else if(RxMessage.StdId == 0x0A2)											//ID of the new gyro
	{
		Gyroscope_Feedback_Decoding_NEW_2(RxMessage);
	}
	else if((RxMessage.StdId & 0x0F0) == 0x0B0)								//ID of the motor is 0x0BX
	//else if(RxMessage.StdId >= 0x0B0 && RxMessage.StdId <= 0x0BF)			//also ok
	{
		Motor_Feedback_Decoding(RxMessage);									//Decode according to the motor ID	
	}
	else if(RxMessage.StdId == 0x0C0)
	{
		Controller_Feedback_Decoding(RxMessage);
	}
	else if(RxMessage.StdId == SEN_BOARD){
		Sensor_Board_Feedback_Decoding(RxMessage);
	}
	else if(RxMessage.StdId == BTN_BOARD){
		Button_Board_Feedback_Decoding(RxMessage);
	}
	else if(RxMessage.StdId == SEN_BAR1){
		Sensor_Bar1_Feedback_Decoding(RxMessage);
	}
	else if(RxMessage.StdId == SEN_BAR2){
		Sensor_Bar2_Feedback_Decoding(RxMessage);
	}
	#else
	if(RxMessage.StdId == 0x0A0)											//ID of the gyro
	{
		Gyro_Cmd_Decoding(RxMessage);
	}
	else if((RxMessage.StdId & 0x0F0) == 0x0B0)								//ID of the motor is 0x0BX
	//else if(RxMessage.StdId >= 0x0B0 && RxMessage.StdId <= 0x0BF)			//also ok
	{
		Motor_Cmd_Decoding(RxMessage);									//Decode according to the motor ID	
	}
	#endif
	#ifdef SENSOR_BOARD
	else if(RxMessage.StdId == SEN_BOARD){
		Sensor_Board_Cmd_Decoding(RxMessage);
	}
	#endif
} 

/* CAN Rx message dequeue and processing ---------------------------------*/
u8 CAN_Rx_dequeue(void)
{
	CanRxMsg RxMsg;
	if(Rx_QueueCounter != 0)
	{
		RxMsg = Rx_Queue[Rx_QueueHead];
		Rx_QueueHead++;
		if(Rx_QueueHead == RX_QUEUE_LENGTH)
			Rx_QueueHead = 0;
		Rx_QueueCounter--;
		CAN_Rx_Processing(RxMsg);
		return 1;
	}
	else
		return CAN_QUEUE_EMPTY;			
}

void CAN_Tx_update(void)
{
	while(CAN_Tx_dequeue() != CAN_QUEUE_EMPTY);
}

void CAN_Rx_update(void)
{
	while(CAN_Rx_dequeue() != CAN_QUEUE_EMPTY);
}

/*Interrupt is handled here*/

void USB_LP_CAN_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	//CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	if(RxMessage.IDE == CAN_ID_STD)
	CAN_Rx_Processing(RxMessage);
}


/*=================== USER INTERFACE ====================*/
void Motor_Set_Vel(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag)
{
	CanTxMsg TxMsg;
	TxMsg = Motor_Velocity_Encoding(Motor_ID, vel, Closed_Loop_Flag);
	CAN_Tx_addToQueue(TxMsg);
}

void Motor_Set_Pos(u32 Motor_ID, u16 vel, s32 pos)
{
	CanTxMsg TxMsg;
	TxMsg = Motor_Position_Encoding(Motor_ID, vel, pos);
	CAN_Tx_addToQueue(TxMsg);
}

void Motor_Set_Parameter(u32 Motor_ID, u16 accel)
{
	CanTxMsg TxMsg;
	TxMsg = Motor_Parameter_Encoding(Motor_ID, accel);
	CAN_Tx_addToQueue(TxMsg);
}

void Motor_Lock(u32 Motor_ID)
{
	CanTxMsg TxMsg;
	TxMsg = Motor_Lock_Encoding(Motor_ID);
	CAN_Tx_addToQueue(TxMsg);
}

void Gyro_Stop_Calibration() {
	CanTxMsg TxMsg;
	TxMsg = Gyro_Stop_Calibration_Encoding();
	CAN_Tx_addToQueue(TxMsg);
}

void Gyro_Reset_Position(s16 x,s16 y,s16 angle)	{
	CanTxMsg TxMsg;
	TxMsg = Gyro_Set_Pos_Encoding(x,y,angle);
	CAN_Tx_addToQueue(TxMsg);
}


void Sensor_Send_Code(u8 code){
	u16 newCode = code;
	CanTxMsg TxMsg;
	newCode<<=1;
	newCode|=1;
	newCode|=1<<10;
	TxMsg=Sensor_Board_Light_Encoding(newCode);
	CAN_Tx_addToQueue(TxMsg);
}

#ifdef SENSOR_BOARD
void sensor_update(){
	u8 i;
	u8 tmp;
	CanTxMsg TxMsg;
	sensor_state=0;
	sensor_state|=!GPIO_ReadInputDataBit(Sensor_Board_SIG_PORT_Array[0],Sensor_Board_SIG_Pin_Array[0]);
	for(i=1;i<NUM_OF_SIG;i++){
		sensor_state<<=1;
		sensor_state|=!GPIO_ReadInputDataBit(Sensor_Board_SIG_PORT_Array[i],Sensor_Board_SIG_Pin_Array[i]);
	}
	TxMsg = Sensor_Board_Sensor_Encoding();
	CAN_Tx_addToQueue(TxMsg);
}
u32 sum1=0;
u32 sum2=0;
u32 sum3=0;
u32 sum4=0;
u8 ADC_Count=0;

void laser_update(){
	  CanTxMsg TxMsg1;
	  CanTxMsg TxMsg2;
		sum1+=ADC_ConvertedValue[0];
		sum2+=ADC_ConvertedValue[1];
		sum3+=ADC_ConvertedValue[2];
		sum4+=ADC_ConvertedValue[3];
		ADC_Count++;
		if(ADC_Count==10){
			ADC_Count=0;
			laser_state1 = 200.0+2*(sum1/10)*0.0575615-13;
			laser_state2 = 200.0+2*(sum2/10)*0.0575615-13;
			laser_state3 = 300.0+(sum3/10)*0.0575615-13;
			laser_state4 = 1400.0+(sum4/10)*0.0575615-13;
			sum1=0;sum2=0;sum3=0;sum4=0;
			TxMsg1 = Sensor_Board_Laser_Encoding1();
			TxMsg2 = Sensor_Board_Laser_Encoding2();
			CAN_Tx_addToQueue(TxMsg1);
			CAN_Tx_addToQueue(TxMsg2);
		}
}
#endif

void Cylinder_Set_State(u8 Cyl_id, u8 state){
	CanTxMsg TxMsg;
	TxMsg = Sensor_Board_Cylinder_Encoding(Cyl_id,state);
	CAN_Tx_addToQueue(TxMsg);
}

#ifdef BUTTON_BOARD
void Button_Board_Update(){
	u8 i;
	u8 tmp;
	CanTxMsg TxMsg;
	sensor_state=0;
	for(i=0;i<20;i++){
		tmp = !GPIO_ReadInputDataBit(Button_Board_PORT_Array[i],Button_Board_Pin_Array[i]);
		button_state|=tmp;
		button_state<<=1;
	}
	tmp = !GPIO_ReadInputDataBit(Button_Board_PORT_Array[i],Button_Board_Pin_Array[i]);
	button_state|=tmp;
	TxMsg = Button_Board_Sensor_Encoding();
	CAN_Tx_addToQueue(TxMsg);
}
#endif


#ifdef SENSOR_BAR
void Sensor_Bar_Update(){
	CanTxMsg TxMsg;
	TxMsg = Sensor_Bar_Encoding();
	CAN_Tx_addToQueue(TxMsg);
}
#endif


void can_ticks_test(u16 ticks, u16 seconds) 
{
	Data_Field data;
	CanTxMsg TxMsg;
	data.Data_Length = 2;
	data.Data[0] = (u8) (ticks & 0xFF);								//indicates the type of cmd
	data.Data[1] = (u8) (seconds & 0xFF);
	TxMsg = General_Encoding(0xAB, data);
	CAN_Tx_addToQueue(TxMsg);
	
	
}