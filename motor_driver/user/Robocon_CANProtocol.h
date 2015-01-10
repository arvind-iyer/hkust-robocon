#ifndef _CAN_PROTOCOL_H
#define _CAN_PROTOCOL_H

#include "Robocon_CAN.h"
#include "pid_vel.h"
#include "pid_pos.h"

#include "uart.h"
//#define GYRO_POSITION_FEEDBACK 		0	
//#define GYRO_VELOCITY_FEEDBACK		0

#define CAN_QUEUE_EMPTY 			0
#define TX_QUEUE_LENGTH				128
#define RX_QUEUE_LENGTH				128

#define RETRY_TIMEOUT				1000

/* CONSTANTS FOR PROTOCOL ***************************************/

/* main control -> motor control */
#define MOTOR_VEL_CMD_LENGTH 		6
#define MOTOR_VEL_CMD 				0xAA

#define MOTOR_POS_CMD_LENGTH 		7
#define MOTOR_POS_CMD 				0xBB

#define MOTOR_PARAMETER_CMD_LENGTH	3
#define MOTOR_PARAMETER_CMD			0x44

#define MOTOR_LOCK_CMD_LENGTH	   	1
#define MOTOR_LOCK_CMD				0xEE	

/* main control -> gyro */
#define GYRO_CAL_CMD_LENGTH			1
#define GYRO_CAL_CMD				0xDD

#define GYRO_SET_CMD_LENGTH			7
#define GYRO_SET_CMD				0xEE

/* motor control -> main control */
#define ENCODER_FEEDBACK_LENGTH		5
#define ENCODER_FEEDBACK			0x22

#define MOTOR_CAL_FEEDBACK_LENGTH	1
#define MOTOR_CAL_FEEDBACK			0xEA

#define MOTOR_ID_OFFSET				0x0B0
#define NUM_OF_MOTORS               16

#define DATA1_FEEDBACK_LENGTH  		5
#define DATA1_FEEDBACK			   	0x55

#define DATA2_FEEDBACK_LENGTH	  	5
#define DATA2_FEEDBACK				0x66

#define DATA3_FEEDBACK_LENGTH		5
#define DATA3_FEEDBACK				0x77

#define DATA4_FEEDBACK_LENGTH		5
#define DATA4_FEEDBACK				0x88

/* gyro -> main control */
#define POS_FEEDBACK_LENGTH			7	
#define POS_FEEDBACK				0x11

#define VEL_FEEDBACK_LENGTH			7	
#define VEL_FEEDBACK				0x33


/* Public variables ------------------------------------------*/
extern s32 Encoder_Count[NUM_OF_MOTORS];			//encoder feedback from the motor controller
extern s16 Robot_Coordinate_X;						//position feedback from the position board
extern s16 Robot_Coordinate_Y;
extern s16 Robot_Angle;
extern s16 Robot_Velocity_X;						//real velocity feedback from the position board
extern s16 Robot_Velocity_Y;
extern s16 Robot_Angular_Velocity;
//test only
extern u32 receive_counter;	   		
extern s32 test_motor_vel;
extern u8 test_motor_flag;
extern u16 test_motor_vel_mag;
extern s32 test_motor_pos;	   

extern u8 Tx_QueueCounter;
extern u8 Rx_QueueCounter;

extern s32 test_data1;
extern s32 test_data2;
extern s32 test_data3;
extern s32 test_data4;	

/* Struct definition ------------------------------------------*/
typedef struct
{
	u8 Data_Length;
	u8 Data[8];									//8 is the max data length of a can data frame
} Data_Field;

/* Functions declaration --------------------------------*/
s32 Four_Bytes_Reconstruction(const uint8_t* buffer);
s16 Two_Bytes_Reconstruction(const uint8_t* buffer);
u8 Four_Bytes_Decomposition(const s32 data, const u8 index);
u8 Two_Bytes_Decomposition(const u16 data, const u8 index);

CanTxMsg General_Encoding(u32 Device_ID, Data_Field Cmd_Data);
CanTxMsg Motor_Velocity_Encoding(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
CanTxMsg Motor_Position_Encoding(u32 Motor_ID, u16 vel, s32 pos);

void Motor_Feedback_Decoding(CanRxMsg RxMessage);
void Gyroscope_Feedback_Decoding(CanRxMsg RxMessage);

void CAN_Tx_addToQueue(CanTxMsg TxMessage);
u8 CAN_Tx_dequeue(void);

void CAN_Rx_addToQueue(CanRxMsg RxMessage);
void CAN_Rx_Processing(CanRxMsg RxMessage);
u8 CAN_Rx_dequeue(void);

void CAN_Tx_update(void);
void CAN_Rx_update(void);

void USB_LP_CAN1_RX0_IRQHandler(void);

void Motor_Set_Vel(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
void Motor_Set_Pos(u32 Motor_ID, u16 vel, s32 pos);
void Send_Encoder(s32 Data);
void Calibration_Done(void);

void Motor_Set_Test(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);

void debug_data1(s32 data);
void debug_data2(s32 data);
void debug_data3(s32 data);
void debug_data4(s32 data);

#endif



