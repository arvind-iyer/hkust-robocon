/**********************************************************************
*				HKUST ROBOCON 2013 CAN PROTOCOL						
*		
*Author: ZHANG Linguang
*Date: January, 2013			
*Description: This file defines the CAN communication protocol
			  between the main control and the sub-systems.
*Updates:
*Added some test funcitions -- Feb, 2013
*Integrated the protocol for all the devices in this file -- Feb, 2013
***********************************************************************/

#include "Robocon_CANProtocol.h"

/* Public variables ------------------------------------------*/
s32 Encoder_Count[NUM_OF_MOTORS] = {0};			//encoder feedback from the motor controller
s16	Robot_Coordinate_X = 0;						//position feedback from the position board
s16 Robot_Coordinate_Y = 0;
s16 Robot_Angle = 0;
s16 Robot_Velocity_X = 0;						//real velocity feedback from the position board
s16 Robot_Velocity_Y = 0;
s16 Robot_Angular_Velocity = 0;
//test only
u32 receive_counter = 0;
s32 test_motor_vel = 0;
u8 test_motor_flag = 0;
u16 test_motor_vel_mag = 0;
s32 test_motor_pos = 0;	   

//for zhaozhetuo
s32 test_data1 = 0;
s32 test_data2 = 0;
s32 test_data3 = 0;
s32 test_data4 = 0;	
	
/* Private variables -----------------------------------------*/
CanTxMsg Tx_Queue[TX_QUEUE_LENGTH];
u8 Tx_QueueHead = 0;
u8 Tx_QueueTail = 0;
u8 Tx_QueueCounter = 0;
CanRxMsg Rx_Queue[RX_QUEUE_LENGTH];
u8 Rx_QueueHead = 0;
u8 Rx_QueueTail = 0;
u8 Rx_QueueCounter = 0;

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


/* =========================================== DATA ENCODING PART ============================================== */


/**********************************************************************
*				Main Control Command Encoding
*Description: These functions encode the command that the main control 
*				send to the other devices
*Variables explanation:
*Device ID: target device ID, eg: we allocate 0xB0~0xBF to motors
*Cmd_Data: we encode the data field of each cmd separately and use
*      		the function "General_Encoding" to generate the final
*			CanTxMsg
***********************************************************************/

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

CanTxMsg Gyro_Calibration_Encoding()
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = GYRO_CAL_CMD_LENGTH;	
	Raw_Data.Data[0] = GYRO_CAL_CMD;
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

CanTxMsg Cal_Feedback_Encoding(u32 Motor_ID)
{
	Data_Field Raw_Data;
	Raw_Data.Data_Length = MOTOR_CAL_FEEDBACK_LENGTH;
	Raw_Data.Data[0] = MOTOR_CAL_FEEDBACK;
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

/* =========================================== DATA ENCODING PART ============================================== */

/**********************************************************************
*			Main Control Command Decoding --- Motor Contorl
*Description: These functions decode the cmd that the main control 
*				send to the motor control
*Variables explanation:
***********************************************************************/
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
  extern u8 cali_done;

	switch(RxMessage.Data[0])
	{
		case MOTOR_VEL_CMD:
			#warning
			uart_tx(COM1, "CAN_OK!\r\n");
			for(i = 0; i < 4; i++)
				tmp1[i] = RxMessage.Data[i+1];
			recombined_vel = Four_Bytes_Reconstruction(tmp1);
			closed_loop_flag = RxMessage.Data[5];
			//CALL ZHAOZHETUO'S FUNCTION
			//SET_VEL(VEL);
	
				if (closed_loop_flag){
	        motor_set_speed((float)recombined_vel);
				}else{ 
					if(cali_done){
					  motor_set_pwm_current((float)recombined_vel);
					}else{
						motor_set_pwm((float)recombined_vel);
					}
				}
			

			test_motor_vel = recombined_vel;
			test_motor_flag = closed_loop_flag;
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

				motor_set_position(recombined_pos,recombined_vel_mag);

			test_motor_vel_mag = recombined_vel_mag;
			test_motor_pos = recombined_pos;
			break;

		case MOTOR_LOCK_CMD:
			motor_lock();
			break;

		case MOTOR_PARAMETER_CMD:
			for(i = 0; i < 2; i++)
				tmp1[i] = RxMessage.Data[i+1];
			accel_val = Two_Bytes_Reconstruction(tmp1);
			//CALL ZHAOZHETUO'S FUNCTION
			set_acceleration(accel_val);
			break;

		default:
			break;
	}
}

/**********************************************************************
*			Main Control Command Decoding --- Position System
*Description: These functions decode the cmd that the main control 
*				send to the position system
*Variables explanation:
***********************************************************************/
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
		case GYRO_CAL_CMD:
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

/* Feedback Decoding ----------------------------------------------------*/

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
	for(i = 0; i < 4; i++)
	{
		tmp[i] = RxMessage.Data[i+1];
	}
	

	
	feedback = Four_Bytes_Reconstruction(tmp);
	if(RxMessage.Data[0] ==  ENCODER_FEEDBACK)
	{
		Encoder_Count[RxMessage.StdId - MOTOR_ID_OFFSET] = feedback;
	}
	else if(RxMessage.Data[0] ==  DATA1_FEEDBACK)
	{
		test_data1 = feedback;	
	}
	else if(RxMessage.Data[0] ==  DATA2_FEEDBACK)
	{
	   	test_data2 = feedback;
	}
	else if(RxMessage.Data[0] ==  DATA3_FEEDBACK)
	{
	 	test_data3 = feedback;
	}
	else if(RxMessage.Data[0] ==  DATA4_FEEDBACK)
	{
	  	test_data4 = feedback;
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
}

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
	#ifdef MAIN_CONTROL
	if(RxMessage.StdId == 0x0A0)											//ID of the gyro
	{
		Gyroscope_Feedback_Decoding(RxMessage);
	}
	else if((RxMessage.StdId & 0x0F0) == 0x0B0)								//ID of the motor is 0x0BX
	//else if(RxMessage.StdId >= 0x0B0 && RxMessage.StdId <= 0x0BF)			//also ok
	{
		Motor_Feedback_Decoding(RxMessage);									//Decode according to the motor ID	
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

/*中断在这里处理*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	//CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

	if(RxMessage.IDE == CAN_ID_STD)
	#ifdef MAIN_CONTROL
	CAN_Rx_addToQueue(RxMessage);
	#else
	CAN_Rx_Processing(RxMessage);
	#endif		
}

/*User interface ---------------------------------------------------------*/
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

/*Testing functions*/
void Motor_Set_Test(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag)
{
	CanTxMsg TxMsg;
	u8 Tx_MailBox = 0;
	u16 retry = RETRY_TIMEOUT;
	TxMsg = Motor_Velocity_Encoding(Motor_ID, vel, Closed_Loop_Flag);

	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);	//wait for the transmission to be finished
}

void Send_Encoder(s32 Data)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Encoder_Feedback_Encoding(MOTOR_ID, Data);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void Calibration_Done(void)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Cal_Feedback_Encoding(MOTOR_ID);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void debug_data1(s32 data)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Data1_Feedback_Encoding(MOTOR2, data);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void debug_data2(s32 data)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Data2_Feedback_Encoding(MOTOR2, data);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void debug_data3(s32 data)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Data3_Feedback_Encoding(MOTOR2, data);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void debug_data4(s32 data)
{
	CanTxMsg TxMsg;
	u16 retry = RETRY_TIMEOUT;
	u8 Tx_MailBox = 0;
	TxMsg = Data4_Feedback_Encoding(MOTOR2, data);
	CAN_Transmit(CAN1, &TxMsg);
	Tx_MailBox = CAN_Transmit(CAN1, &TxMsg);							//transmit the message
	while(CAN_TransmitStatus(CAN1,Tx_MailBox) != CANTXOK && retry--);
}

void Test_Receive()
{
	
}






