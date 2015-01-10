#include "pid_vel.h"

u8 enable_bar = 0;
u8 enable_bar_memory=0;
float kp1 = 40;
float ki1 = 0.8;		
float kd1 = 0;

float kp2 = 40;
float ki2 = 0.8;
float kd2 = 0;

float ip = 3;
float ii = 0.2;
float id = 0;

float cal_P = 0;
float cal_I = 0;
float cal_D = 0;
float PID = 0;

float err_curr=0;
float err_prev1=0;
float err_prev2=0;
float err_total=0;


//counting the encoder
float count=0;

float r_count = 0; //desired counts every sampling period
float r_count_prev = 0;

float d_count = 0; //actual counts every sampling period
float d_count_prev = 0;
float d_cal=0;
float d_carrier=0;
u16 memory_index=0;
float dcount_memory[MEMORY_SIZE];


extern float max_pwm;

s8 dir = 0;
float pwm = 0;
float pwm_init = 0;

float set_pwm = 0;
s8 set_dir = 0;

extern u8 reach_token;

u8 vel_reach=0;
extern float ticks_counter;
float counter_memory=0;
u16 counter_index=0;


float err_curr_pos=0;
float err_prev1_pos=0;
float err_prev2_pos=0;
float pos_init=0;


//float max_delta=100000;


// command only for debug
void vel_set_pid1(float _p, float _i, float _d)
{
	kp1 = _p;
	ki1 = _i;
	kd1 = _d;	
}	

// command only	for debug
void vel_set_pid2(float _p, float _i, float _d)
{
	kp2 = _p;
	ki2 = _i;
	kd2 = _d;	
}	

// command only	for debug
void vel_init_pwm(float _pwm, s8 _dir)
{
 	pwm = _pwm;
	dir = _dir;
}

// command only	for debug
void vel_set_max(float m)
{
 	max_pwm = m;
}

// command only	for debug
void vel_set_speed(float r)
{
	r_count = r;
}

// command only	for debug
float vel_get_speed(void)
{
	return d_count; 
}


// sub component
void clear_record(void)
{
    err_curr=0;
	err_prev1=0;
	err_prev2=0;

	err_curr_pos=0;
	err_prev1_pos=0;
	err_prev2_pos=0;

	cal_P=0;
	cal_I=0;
	cal_D=0;
	PID=0;

	pwm=0;

	reach_token = 0;
}

void init_memory(void)
{
	int i=0;
	for (i=0; i<MEMORY_SIZE; i++)
	{
		dcount_memory[i]=0;
	}
}

// sub component
void read_encoder(void)
{
	count = TIM_GetCounter(TIM2);
	if (count>62768)
	{
		s32 offset=count-30000;
		count=offset;
		TIM_SetCounter(TIM2, offset);
		d_carrier++;
	}
	else if (count<2768)
	{
		s32 offset=count+30000;
		count=offset;
		TIM_SetCounter(TIM2, offset);
		d_carrier--;
	}
	
	d_cal =	d_carrier*30000+count-32768;				
	d_count = -d_cal + dcount_memory[memory_index];

	dcount_memory[memory_index]=d_cal;
	
	if (memory_index>MEMORY_SIZE-2)
		memory_index=0;
	else
		memory_index++;
}

void vel_err(void)
{
    err_curr = r_count - d_count;			
}


//sub component
float current_input=0;
void vel_calculate_pwm(float p, float i, float d)
{

	cal_P = p * (err_curr - err_prev1);
	cal_I = i * err_curr;
	cal_D = d * (err_curr + err_prev2 - 2*err_prev1);

	err_prev2 = err_prev1;
	err_prev1 = err_curr;
		
	PID = cal_P + cal_I + cal_D;

	current_input += PID;

	if(current_input > MAX_CURRENT)	   current_input = MAX_CURRENT;
	if(current_input < -MAX_CURRENT)   current_input = -MAX_CURRENT;

}


//sub component
void set_pwm_to_motor(void)
{	
    if (pwm>=0)
	    motion_set_motor(pwm,dir);
    else  
	    motion_set_motor((-1)*pwm,dir);
}

float ADC_offset=0;
u16 tim_counter=0;
u8 cali_done=0;
extern vu16  ADC_ConvertedValue[];
void ADC_calibration(void)
{
	if (!cali_done)
	{
		if (tim_counter<5000)
		{
			ADC_offset+=ADC_ConvertedValue[0];
			tim_counter++;
		}
		else 
		{
			ADC_offset/=5000;
			cali_done=1;
		}
	 }
}

float histogram[7]={0,0,0,0,0,0,0};
extern vu16 ADC_ConvertedValue[];
float I_cal_P = 0;
float I_cal_I = 0;
float I_cal_D = 0;
float I_PID = 0;

float I_err_curr=0;
float I_err_prev1=0;
float I_err_prev2=0;
void current_circle(float p, float i, float d)
{
	I_err_curr=current_input-(ADC_ConvertedValue[0]-ADC_offset);

    I_cal_P = p * (I_err_curr - I_err_prev1);
	I_cal_I = i * I_err_curr;
	I_cal_D = d * (I_err_curr + I_err_prev2 - 2*I_err_prev1);

	I_err_prev2 = I_err_prev1;
	I_err_prev1 = I_err_curr;
		
	I_PID = I_cal_P + I_cal_I + I_cal_D;

	pwm = pwm + I_PID;

	dir = (pwm>0)?1:0;

	if(pwm > MAX_PWM)	pwm = MAX_PWM;
	if(pwm < -MAX_PWM)   pwm = -MAX_PWM;

}	

void vel_move(void)
{
	vel_err();

 	vel_calculate_pwm(kp1,ki1,kd1);

//	current_input=r_count;

	current_circle(ip,ii,id);
}

//sub function
void vel_stop(void)
{  
	r_count=0;

	vel_err();

	vel_calculate_pwm(kp2,ki2,kd2);
	
	current_circle(ip,ii,id);
} 



//sub function
void vel_zero(void)
{
	pwm=0;
	dir=0;
}

//sub function
void vel_pwm(void)
{
	pwm=set_pwm;
	dir=set_dir;
}


// main function
void vel_n_pos(void)
{
    read_encoder();
	if (cali_done)
	{
		if (enable_bar==0)  
			vel_zero();
	
		else if(enable_bar==2)
			vel_stop();
		
		else if(enable_bar==1) 
			vel_move();
		
		else if(enable_bar==3)
			pos_move();
	
		else if(enable_bar==5) 
			vel_pwm();
	}
	else if(enable_bar==6) 
		ADC_calibration();

	set_pwm_to_motor();
	
}

// function	called by user
void motor_set_speed(float speed, s8 direction)
{
    enable_bar=1;
    if(direction==1)	r_count=speed;
	else if(direction==0)  r_count=-speed;
}
	
// function	called by user
void motor_lock()
{
    enable_bar=2;
}

// function	called by user
void motor_zero()
{
    enable_bar=0;
}








