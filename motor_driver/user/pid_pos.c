#include "pid_pos.h"
#include "math.h"

float kp_pos = 0;
float ki_pos = 0;
float kd_pos = 0;

//need initial values
float max_acc= 3;  			//10
float max_pwm= 300;  //max speed
float min_ac= 0;

float count_total = 0;
float count_init = 0;
extern float d_cal;

float predictor=0;
float predict_parameter=0;  //10

float count_dest = 0;
float count_dest_p = 0;
extern float pwm;
float prev_pwm=0;
extern u8 dir;

float count_input = 0;
float encoder_count;
extern float r_count;
extern float d_count;


float count_input;
u8 reach_token=0;

u8 mode_bar=0;
extern u16 ticks;
extern u8 enable_bar;

float offset;
float sqrt_rt1;
float sqrt_rt2;

float abs(float input)
{
    if (input>=0)
	    return input;
	else 
	    return -input;
}

//is a command
void pos_set_pid(float _p, float _i, float _d)
{

}

//is a command
void pos_set_max(float _acc, float _pwm)
{
	max_acc = _acc;
	max_pwm = _pwm;
}

//is a command
float pos_get_curr(void)
{
 	return count_total;
}

//is a command
void pos_set_home(void)
{
	count_total = 0;
	count_dest = 0;
//	pwm=0;
	reach_token = 0;
	predictor=0;
	count_init = d_cal;
}

void pos_err(void)
{
   
}

float max_speed;

void pos_cal_function_mode(void)
{
	count_total= -d_cal+count_init;

	predictor = count_total + d_count*predict_parameter;		

	if (count_dest>=0)
	{
	    offset=5;
		sqrt_rt1=sqrt(abs(2*predictor*max_acc));
		sqrt_rt2=sqrt(abs(2*(count_dest-predictor)*max_acc));	
		max_speed=max_pwm;
	}
	else 
	{
	    offset=-5;
		sqrt_rt1=-sqrt(abs(2*predictor*max_acc));
		sqrt_rt2=-sqrt(abs(2*(count_dest-predictor)*max_acc));
		max_speed=-max_pwm;
	}

	if (abs(count_dest) <= abs(max_pwm*max_pwm/max_acc))
	{	
	    if (abs(predictor) == 0)
		    count_input=offset;

	    else if (abs(predictor) <= abs(count_dest/2))
		    count_input=sqrt_rt1+offset;

		else if (abs(predictor)>abs(count_dest/2) && abs(predictor)<abs(count_dest))
		    count_input=sqrt_rt2+offset; 

		else 
		{
		    reach_token=1;
			count_input=0;
		}
	}
	else 
	{
	    if (abs(predictor) == 0)
		    count_input=offset;

	    else if (abs(predictor) < abs(max_pwm*max_pwm/(2*max_acc)))
			count_input=sqrt_rt1+offset;

		else if  (abs(predictor) >= abs(max_pwm*max_pwm/(2*max_acc)) 
		       && abs(predictor) <= abs(abs(count_dest)-(max_pwm*max_pwm/(2*max_acc))))
			count_input=max_speed+offset;

		else if  ( abs(predictor) > abs(abs(count_dest)-(max_pwm*max_pwm/(2*max_acc))) 
		       && abs(predictor) < abs(count_dest))			   
			count_input=sqrt_rt2+offset;
		else 
		{
		    reach_token=1;
			count_input=0;
		}
	}

	r_count=count_input;

    	
}	 



void pos_move_function_mode(u8 mode)
{
    if (ticks%10==0) 
	    pos_cal_function_mode();

	vel_move(mode);
}

void pos_move(u8 mode)
{
	if (abs(count_dest) > abs(predictor))
		pos_move_function_mode(mode);
	else 
		enable_bar=2;
	    
}

extern u8 mode;
//called by user
void motor_set_position(s32 position, u16 vel)
{
	pos_set_home();
	max_pwm=vel;
	count_dest=position;

//	mode=4;
	enable_bar=3;
}

