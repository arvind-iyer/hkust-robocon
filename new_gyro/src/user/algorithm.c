/*************ROBOCON 2012 POSITION SYSTEM******************/
/*********************VERSION 0.4***************************/
#include "algorithm.h"

//This algorithm.c consists of five independent algorithms for localization

//ENCODER VALUE------------------------------------
static volatile s32 curr_encoder_pos[3] = { 0 , 0 , 0}; //used as a buffer for current encoder sampling (originally 0)
static volatile s32 prev_encoder_pos[3] = { 0 , 0 , 0}; //buffer which stores the previous encoder data (originally 0)
static volatile s32 vel_buf[3][BUF_LEN]; //GYRO has delay, so we delay the velocity to synchronize the speed
static volatile u8 head = 0;
static volatile u8 tail = BUF_LEN - 2;
static volatile u8 prev = BUF_LEN - 1;
//-------------------------------------------------
static volatile s32 curr_ang_encoder_pos = 0;
static volatile s32 ang_encoder_value = 0;
static volatile s32 curr_encoder_angle = 0;
static volatile s32 prev_encoder_angle = 0;
static volatile s32 encoder_value = 0;
static volatile s32 adjust_encoder_angle = 0;
static volatile s32 abs_encoder_X = 0;
static volatile s32 abs_encoder_Y = 0;
static volatile s16 encoder_X = 0;
static volatile s16 encoder_Y = 0;

// gyro
static volatile float ang_vel = 0;

//-------------------------------------------------
static volatile s32 adjusted_gyro_angle = 0;
static volatile s32 curr_gyro_angle = 0;
static volatile s32 prev_gyro_angle = 0;
volatile s16 X_OFF = 0;
volatile s16 Y_OFF = 0;
volatile s16 ANGLE_OFF = 0;
//Position value-----------------------------------
static volatile double curr_vel_X = 0;
static volatile double curr_vel_Y = 0;
static volatile double prev_vel_X = 0;
static volatile double prev_vel_Y = 0;
static volatile double abs_X = 0;
static volatile double abs_Y = 0;
volatile double curr_angle_r = 0;
volatile double sin_value =0;
volatile double cos_value =0;
volatile s16 real_X = 0;
volatile s16 real_Y = 0;
//System select------------------------------------
volatile u8 system_select = 1;					//**default is T-SHAPE

//For calculation
static volatile float linear_vel = 0;
static volatile float y_displacement_offset = 0;		//accumulated by turning and rotating

//FOR DEBUG
volatile s16 non_scaled_gyro = 0;
volatile s16 gyro_velocity = 0;
volatile s16 en_vel0 = 0;
volatile s16 en_vel1 = 0;
volatile s16 omega_r = 0;


double offset_factor = 1/(LSB* PI/ 180 * Sampling_time * RADIUS / Encoder_Circumference * Encoder_Count );
double encoder_factor = Encoder_Circumference / Encoder_Count;

/******************************************FUNCTIONS*************************************************/

void calculation(u8 system )
{
	u8 i = 0;
	
	//-------GET ENCODER VALUE-----------------
	read_encoders((s32*)curr_encoder_pos);//Read all the encoders datas according to system_select
									//**curr_encoder_pos is the buffer which stores the total
									//distance the 3 encoders have traveled respectively
									//velocity is calculated by doing subtraction (next step)

	for(i = 0; i<3; i++)
		vel_buf[i][tail] = curr_encoder_pos[i] - prev_encoder_pos[i]; 	//**calculate the displacement-->scaled velocity 
																	//store in velocity buffer


	//-------FILTER FOR ONE ENCODER SYSTEM------
	if(system_select == ONE_ENCODER_SYSTEM)
	{
		curr_ang_encoder_pos = curr_encoder_pos[1];	
		if(ang_encoder_value - curr_ang_encoder_pos > ENCODER_TH || ang_encoder_value - curr_ang_encoder_pos < -ENCODER_TH)
		ang_encoder_value = curr_ang_encoder_pos;
	}	

		

	/**********************************Calculation******************************/

	switch(system_select)
	{		
		//********ONE_ENCODER_SYSTEM********//
		case ONE_ENCODER_SYSTEM:		
			curr_encoder_angle = -(ang_encoder_value % ENCODER_COUNTS)*ENCODER_ANGLE_SCALE  + real_angle;
			if(curr_encoder_angle < 0)
				curr_encoder_angle += 3600;
			else if(curr_encoder_angle >+ 3600)
				curr_encoder_angle -= 3600;
			adjust_encoder_angle = (curr_encoder_angle + prev_encoder_angle) >> 1;
			
			//Calculate position of the encoder then the robot
			abs_encoder_X -=(s32)vel_buf[0][head]*int_sin(adjust_encoder_angle);        
			abs_encoder_Y +=(s32)vel_buf[0][head]*int_cos(adjust_encoder_angle);			
			encoder_X = (s16)(abs_encoder_X/ENCODER_POS_SCALE);
			encoder_Y = (s16)(abs_encoder_Y/ENCODER_POS_SCALE);						
			real_X = encoder_X + ENCODER_LENGTH * int_sin(curr_encoder_angle)/10000 + X_OFF;
			real_Y = encoder_Y - ENCODER_LENGTH * int_cos(curr_encoder_angle)/10000 + Y_OFF;			
		break;
		
		//********T-SHAPE_SYSTEM*******//
		case T_SHAPE_SYSTEM:
		//--------calculate angular_velocity*R------------//
		//but need to be scaled with encoders at last
			
			y_displacement_offset =  curr_ang_vel / offset_factor;
			curr_angle_r = (real_angle * PI) / 1800; 
			sin_value = sin(-curr_angle_r);
			cos_value = cos(-curr_angle_r);
			curr_vel_Y = (sin_value*vel_buf[0][head] + cos_value*(vel_buf[1][head]-y_displacement_offset));
			curr_vel_X = (cos_value*vel_buf[0][head] - sin_value*(vel_buf[1][head]-y_displacement_offset));
			
			abs_X += (curr_vel_X+prev_vel_X)/2;
			abs_Y += (curr_vel_Y+prev_vel_Y)/2;
				
			real_X = (abs_X*encoder_factor) + X_OFF; //the sequence of operations is to avoid overflow during calculation
			real_Y = (abs_Y*encoder_factor) + Y_OFF;
			
			/*
			ang_vel =  ( curr_ang_vel*100.0 )/GYRO_SCALE;
			linear_vel = ang_vel*RADIUS/RADIAN_CONVERTION;	
																	//but this is the real value, we need to scale it
			y_displacement_offset = linear_vel*ENCODER_1REV_COUNTS/ENCODER_PERIMETER/10;			

			curr_vel_Y = (sys_sin(-real_angle)*vel_buf[0][head] + sys_cos(-real_angle)*(vel_buf[1][head]-y_displacement_offset))/TRIANGLE_FUNCTION_SCALE;
			curr_vel_X = (sys_cos(-real_angle)*vel_buf[0][head] - sys_sin(-real_angle)*(vel_buf[1][head]-y_displacement_offset))/TRIANGLE_FUNCTION_SCALE;
				
			abs_X += (curr_vel_X+prev_vel_X)/2;
			abs_Y += (curr_vel_Y+prev_vel_Y)/2;
				
			real_X = (abs_X/100*ENCODER_PERIMETER/200) + X_OFF; //the sequence of operations is to avoid overflow during calculation
			real_Y = (abs_Y/100*ENCODER_PERIMETER/200) + Y_OFF;
			
			*/
			//printf( "%ld %ld %d %d\n" , vel_buf[0][head] , vel_buf[1][head] , curr_ang_vel , real_angle );
			
			//printf( "%ld %ld %d %f %f \r\n" ,vel_buf[0][head] ,vel_buf[1][head] , real_angle , y_displacement_offset  ,curr_vel_Y  );
			//real_X = (s16)(abs_X/100*ENCODER_PERIMETER/ENCODER_1REV_COUNTS) + X_OFF; //the sequence of operations is to avoid overflow during calculation
			//real_Y = (s16)(abs_Y/100*ENCODER_PERIMETER/ENCODER_1REV_COUNTS) + Y_OFF;
			//printf( "%ld,%ld\r\n" , abs_X , abs_Y );
		break;
		
	}
	
	//Queue-------------------------------		//**this part is updating the old values with current values											//prepare for the next loop
	tail = (tail + 1) % BUF_LEN;
	head = (head + 1) % BUF_LEN;
	prev = (prev + 1) % BUF_LEN;

	//Prev State Data---------------------
	prev_vel_X = curr_vel_X;
	prev_vel_Y = curr_vel_Y;
	
	//Prev Encoder Data-------------------
	prev_encoder_pos[0] = curr_encoder_pos[0];
	prev_encoder_pos[1] = curr_encoder_pos[1];
	prev_encoder_pos[2] = curr_encoder_pos[2];
}

//set the current position of position system

void position_set(s16 x, s16 y, s16 angle)
{
	s8 i =0 , j = 0;	
	if(system_select>1) 	//L-SHAPE H-SHAPE 
		position_set_still_encoder(angle,x,y);
	else
		position_set_joint_encoder(angle,x,y);
	
	for(i=0;i<15;i++)
	{
		for(j= 0; j<2; j++)
			vel_buf[j][i] = 0;		//Clear velocity buffer
	}	
}


//status_set.....I don't know the usage....

void status_set(u8 status, s16 angle, s16 x, s16 y)
{
	s8 i = 0 , j = 0;
	if(system_select>1)
		status_set_still_encoder(status,angle,x,y);
	else
		status_set_joint_encoder(status,angle,x,y);
	
	for( i=0;i<15;i++)
	{
		for( j= 0; j<2; j++)
			vel_buf[j][i] = 0;
	}
}


void status_set_joint_encoder(u8 status, s16 angle, s16 x, s16 y)
{
    if (status == 1)
    {
        X_OFF = x;
        Y_OFF = y;
        ANGLE_OFF = angle;
        sim_angle = -ANGLE_OFF;
    }
}

void position_set_joint_encoder(s16 angle, s16 x, s16 y)
{
	set_angle( angle );
    if (abs(x - real_X) > 0 || abs(y - real_Y) > 0)
    {
		real_X = 0;
		real_Y = 0;
		abs_X = 0; 
		abs_Y = 0;	
		X_OFF = x ;
		Y_OFF = y;
    }
//	printf( "finish" );
}

void status_set_still_encoder(u8 status, s16 angle, s16 x, s16 y)
{
    if (status == 1)
    {
		real_X = x;
        real_Y = y;
		//abs_X = (ENCODER_COUNTS/ENCODER_PERIMETER)*real_X;
		//abs_Y = (ENCODER_COUNTS/ENCODER_PERIMETER)*real_Y;
		sim_angle = angle;
		gyro_angle = sim_angle*GYRO_SCALE;
    }
}

void position_set_still_encoder(s16 angle, s16 x, s16 y)
{	
	real_X = x;
	real_Y = y;
//	abs_X = (ENCODER_COUNTS/ENCODER_PERIMETER)*real_X;
//	abs_Y = (ENCODER_COUNTS/ENCODER_PERIMETER)*real_Y;
	sim_angle = angle;
	gyro_angle = sim_angle*GYRO_SCALE;
}

