#ifndef _CAN_PROTOCOL_H
#define _CAN_PROTOCOL_H

#include "Robocon_CAN.h"

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

#define MOTOR_CAL_CMD_LENGTH		0
#define MOTOR_CAL_CMD				0x44

/* main control -> gyro */
#define GYRO_CAL_CMD_LENGTH			1
#define GYRO_CAL_CMD				0xDD

#define GYRO_SET_CMD_LENGTH			7
#define GYRO_SET_CMD				0xEE

/* motor control -> main control */
#define ENCODER_FEEDBACK_LENGTH		5
#define ENCODER_FEEDBACK			0x22
#define MOTOR_ID_OFFSET				0x0B0
#define NUM_OF_MOTORS               16

/* gyro -> main control */
#define POS_FEEDBACK_LENGTH			7	
#define POS_FEEDBACK				0x11

#define VEL_FEEDBACK_LENGTH			7	
#define VEL_FEEDBACK				0x33



/* Devices ID definition --------------------------------*/

//Gyroscope(Position system)
#define POS_SYSTEM 					0x0A0

//Motors, Prepared 16 motors for future use, ID = 0x0BX
#define MOTOR1						0x0B0
#define MOTOR2						0x0B1
#define MOTOR3						0x0B2
#define MOTOR4						0x0B3
#define MOTOR5						0x0B4
#define MOTOR6						0x0B5
#define MOTOR7						0x0B6
#define MOTOR8						0x0B7
#define MOTOR9						0x0B8
#define MOTOR10						0x0B9
#define MOTOR11						0x0BA
#define MOTOR12						0x0BB
#define MOTOR13						0x0BC
#define MOTOR14						0x0BD
#define MOTOR15						0x0BE
#define MOTOR16						0x0BF

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

void USB_LP_CAN_RX0_IRQHandler(void);

void Motor_Set_Vel(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
void Motor_Set_Pos(u32 Motor_ID, u16 vel, s32 pos);

void Motor_Set_Test(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);

#endif



