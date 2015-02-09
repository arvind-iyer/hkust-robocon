#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "delay.h"
#include "system.h"
#include "math.h"

/******************Mech Setting**********************/
//in mm the radius from the two encoders to the center encoder
//you must measure the RADIUS if you are not using ROBOCON's encoders set
//shape of Encoder is  |-|  V2 V0 V1.  R1 is the distance between V0 and V1, R2 is the dis between V0 and V2  
#define RADIUS                  150
#define LSB                     0.07326	      // degree
#define Encoder_Count           2000
#define Encoder_Circumference   160.2212253   //in mm
#define Sampling_time           0.003908

#define PI 3.14159265


//-----------------FOR ONE ENCODER or DUAL AXIS SYSTEM-------------------
#define ENCODER_LENGTH 			130 //in mm
#define ENCODER_POS_SCALE 		162818
#define ENCODER_COUNTS 			6582            //1800*4
#define ENCODER_ANGLE_SCALE 	3600/6582

/******************Software Setting******************/
//1 LSB = 0.07326 deg/sec
//dt = 3.906 msec
#define ENCODER_TH				4
#define BUF_LEN 				17  //BUF_LEN is used due to the delay of the GYRO response this 15 origanally
//#define PC_TEST 0
#define TRIANGLE_FUNCTION_SCALE 1000.0 //(should be 10000 for sin and cos function but for precision it is set to 1000)
#define ONE_ENCODER_SYSTEM 		0
#define T_SHAPE_SYSTEM 			1



extern volatile s32 test_X;
extern volatile s32 test_Y;
extern volatile s32 real_x_scale;
extern volatile s32 real_y_scale;
extern volatile s16 real_X ;
extern volatile s16 real_Y ;


extern volatile u8 system_select;
void calculation(u8 system);
void status_set_joint_encoder(u8 status, s16 angle, s16 x, s16 y);
u8 status_report(void);
void position_set_joint_encoder(s16 angle, s16 x, s16 y);
void status_set_still_encoder(u8 status, s16 angle, s16 x, s16 y);
void position_set_still_encoder(s16 angle, s16 x, s16 y);
void position_set(s16 angle, s16 x, s16 y);
void status_set(u8 status, s16 angle, s16 x, s16 y);

#endif		/* __ALGORITHM_H */
