#ifndef _CAN_PROTOCOL_H
#define _CAN_PROTOCOL_H

#include "can.h"
#include "ticks.h"

#define CAN_QUEUE_EMPTY 			0
#define TX_QUEUE_LENGTH				255
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
#define GYRO_STOP_CAL_CMD_LENGTH	1
#define GYRO_STOP_CAL_CMD			0xDD

#define GYRO_SET_CMD_LENGTH			7
#define GYRO_SET_CMD				0xEE

/* main control -> io board */
#define IO_SET_CMD_LENGTH			5
#define IO_SET_CMD					0x97

/* motor control -> main control */
#define ENCODER_FEEDBACK_LENGTH		5
#define ENCODER_FEEDBACK			0x22

#define MOTOR_CAL_FEEDBACK_LENGTH	1
#define MOTOR_CAL_FEEDBACK			0xEA

#define MOTOR_ID_OFFSET				0x0B0
#define NUM_OF_MOTORS               2

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

#define GYRO_CAL_FEEDBACK_LENGTH    2
#define GYRO_CAL_FEEDBACK           0x99

/* io board -> main control */
#define IO_DATA_LENGTH				8
#define IO_DATA						0x98

/* main <-> sensor board*/
#define SEN_DATA_LENGTH    3
#define SEN_DATA					0x11

#define CYL_DATA_LENGTH			3
#define CYL_DATA					0x22

#define LIGHT_DATA_LENGTH 2
#define LIGHT_DATA				0x33

#define LASER_DATA1_LENGTH 6
#define LASER_DATA1				0x44

#define LASER_DATA2_LENGTH 6
#define LASER_DATA2				0x45

#define BTN_DATA_LENGTH			4
/* sensor board -> main */

/* Public variables ------------------------------------------*/
extern s32 Encoder_Count[NUM_OF_MOTORS];			//encoder feedback from the motor controller
extern s16 Robot_Coordinate_X;						//position feedback from the position board
extern s16 Robot_Coordinate_Y;
extern s16 Robot_Angle;
extern s16 Robot_Velocity_X;						//real velocity feedback from the position board
extern s16 Robot_Velocity_Y;
extern s16 Robot_Angular_Velocity;
extern u8  Clyinder_State[16];
extern u8 final_result;
/***************************/
// check the state of sensor
// usage: sensor_state&xxxx
// e.g. : sensor_state&SENSOR1
/***************************/
extern u16 sensor_state;
#define SENSOR5 2048
#define SENSOR6 1024
#define SENSOR7 512
#define SENSOR8 256
#define SENSOR9 128
#define SENSOR10 64
#define SENSOR11 32
#define SENSOR12 16
#define SENSOR13 8
#define SENSOR14 4
#define SENSOR15 2
#define SENSOR16 1

//extern u32 button_state;
#define BUTTON1 4194304
#define BUTTON2 2097152
#define BUTTON3 1048576
#define BUTTON4 524288
#define BUTTON5 262144
#define BUTTON6 131072
#define BUTTON7 65536
#define BUTTON8 32768
#define BUTTON9 16384
#define BUTTON10 8192
#define BUTTON11 4096
#define BUTTON12 2048
#define BUTTON13 1024
#define BUTTON14 512
#define BUTTON15 256
#define BUTTON16 128
#define BUTTON17 64
#define BUTTON18 32
#define BUTTON19 16
#define BUTTON20 8
#define BUTTON21 4
#define BUTTON22 2
#define BUTTON23 1

extern u32 receive_counter;	 
extern s32 CAN_motor_vel;
extern u8 CAN_motor_flag;
extern u16 CAN_motor_vel_mag;
extern s32 CAN_motor_pos;	   

extern u8 Tx_QueueCounter;
extern u8 Rx_QueueCounter;
extern u8 sensor_bar_result1;
extern u8 sensor_bar_result2;
extern u8 Light_Count;

extern vu16 ADC_ConvertedValue[4];


extern u16 laser_state1;
extern u16 laser_state2;
extern u16 laser_state3;
extern u16 laser_state4;


extern u8 laser_Q1;
extern u8 laser_Q2;
extern u8 laser_Q3;
extern u8 laser_Q4;

//motor cali
extern s32 Motor_Valid[NUM_OF_MOTORS];
	

/* Struct definition ------------------------------------------*/
typedef struct
{
	u8 Data_Length;
	u8 Data[8];									//8 is the max data length of a can data frame
} Data_Field;

/* Functions declaration --------------------------------*/
//s32 Four_Bytes_Reconstruction(const uint8_t* buffer);
//s16 Two_Bytes_Reconstruction(const uint8_t* buffer);
//u8 Four_Bytes_Decomposition(const s32 data, const u8 index);
//u8 Two_Bytes_Decomposition(const u16 data, const u8 index);

CanTxMsg General_Encoding(u32 Device_ID, Data_Field Cmd_Data);
CanTxMsg Motor_Velocity_Encoding(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
CanTxMsg Motor_Position_Encoding(u32 Motor_ID, u16 vel, s32 pos);

CanTxMsg Gyro_Set_Pos_Encoding(s16 Pos_X, s16 Pos_Y, s16 Angle);

void Motor_Feedback_Decoding(CanRxMsg RxMessage);
void Gyroscope_Feedback_Decoding(CanRxMsg RxMessage);
CanTxMsg Pos_Feedback_Encoding_NEW(u16 Angle);

void CAN_Tx_addToQueue(CanTxMsg TxMessage);
u8 CAN_Tx_dequeue(void);

void CAN_Rx_addToQueue(CanRxMsg RxMessage);
void CAN_Rx_Processing(CanRxMsg RxMessage);
u8 CAN_Rx_dequeue(void);

void CAN_Tx_update(void);
void CAN_Rx_update(void);

#ifdef STM_HD
void USB_LP_CAN_RX0_IRQHandler(void);
#else
void USB_LP_CAN1_RX0_IRQHandler(void);
#endif

void Gyro_Send_Pos(u16 Pos_Angle);
void Gyro_Send_Pos_2(s16,s16,u16);


void Motor_Set_Vel(u32 Motor_ID, s32 vel, u8 Closed_Loop_Flag);
void Motor_Set_Pos(u32 Motor_ID, u16 vel, s32 pos);
void Motor_Set_Parameter(u32 Motor_ID, u16 accel);
void Motor_Lock(u32 Motor_ID);
void IO_SetBits(u32 Data);

void Gyro_Stop_Calibration(void);
void Gyro_Reset_Position(s16 x,s16 y,s16 angle);

void sensor_update(void);
void laser_update(void);
void Cylinder_Set_State(u8 Cyl_id, u8 state);
#define CYL_ON 0
#define CYL_OFF 1

void Button_Board_Update(void);

#ifdef SENSOR_BAR
void Sensor_Bar_Update(void);
#endif

void Sensor_Bar1_Feedback_Decoding(CanRxMsg RxMessage);
void Sensor_Bar2_Feedback_Decoding(CanRxMsg RxMessage);
#endif

void can_ticks_test(u16 ticks, u16 seconds);



