#include "pid_vel.h"

u8 enable_bar = 0;
u8 enable_bar_memory=0;
float kp1 = 15; //25;//40;
float ki1 = 0.45; //0.65; //0.8;		
float kd1 = 30; //50; //0

float kp2 = 15; //25;//40;
float ki2 = 0.45; //0.65;//0.8;
float kd2 = 30; //50;//0;

float ip = 2.1;
float ii = 0.14;
float id = 0;

float kp11 = 16; //25;//40;
float ki11 = 0.3; //0.65; //0.8;		
float kd11 = 120; //50; //0

float kp21 = 16; //25;//40;
float ki21 = 0.3; //0.65;//0.8;
float kd21 = 120; //50;//0;

float cal_P = 0;
float cal_I = 0;
float cal_D = 0;
float PID = 0;

float err_curr=0;
float err_prev1=0;
float err_prev2=0;
float err_total=0;

s32 abs_encoder = 0;

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

//float current_memory[10];

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
/*
	for (i=0; i<10; i++)
	{
		current_memory[i]=0;
	}
*/
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

float increment=0.5;
float user_speed=0;
u8 cali_done=0;

void vel_err(void)
{
	if (user_speed-r_count>increment)
		r_count+=increment;
	else if (user_speed-r_count<-increment)
		r_count-=increment;
	else r_count=user_speed;

    err_curr = r_count - d_count;		

if(cali_done){
	if(r_count < 30 && r_count > -30){
    kp1 = 15;
    kp2 = 15;
    ki1 = 0.45;
		ki2 = 0.45;
		kd1 = 30;
		kd2 = 30;
		ip = 2.1;
		ii = 0.14;
		id = 0;
			GPIO_WriteBit(GPIOC,GPIO_Pin_0,1);
			GPIO_WriteBit(GPIOC,GPIO_Pin_1,0);
	}else{
		kp1 = 8;
    kp2 = 8;
    ki1 = 0.08;
		ki2 = 0.08;
		kd1 = 32;
		kd2 = 32;
		ip = 3;
		ii = 0.2;
		id = 0;
		GPIO_WriteBit(GPIOC,GPIO_Pin_1,1);
		GPIO_WriteBit(GPIOC,GPIO_Pin_0,0);
	}	
}	
}



//sub component
float current_input=0;
void vel_calculate_pwm(float p, float i, float d, u8 mode)
{

	cal_P = p * (err_curr - err_prev1);
	cal_I = i * err_curr;
	cal_D = d * (err_curr + err_prev2 - 2*err_prev1);

	err_prev2 = err_prev1;
	err_prev1 = err_curr;
		
	PID = cal_P + cal_I + cal_D;

	if (mode==0)
	{
		current_input += PID;
	
		if(current_input > MAX_CURRENT)	   current_input = MAX_CURRENT;
		if(current_input < -MAX_CURRENT)   current_input = -MAX_CURRENT;
	}
	else
	{
		pwm = pwm + PID;

		dir = (pwm>0)?1:0;					  

		if(pwm > MAX_PWM)	pwm = MAX_PWM;
		if(pwm < -MAX_PWM)   pwm = -MAX_PWM;
	}
}


//sub component
void set_pwm_to_motor(void)
{	
    if (pwm>=0)
	    motion_set_motor(pwm,dir);
    else  
	    motion_set_motor((-1)*pwm,dir);
}

float ADC_offset[2];
u16 tim_counter=0;

extern vu16  ADC_ConvertedValue[];
void ADC_calibration(void)
{
	if (!cali_done)
	{
		if (tim_counter<5000)
		{
			ADC_offset[0]+=ADC_ConvertedValue[0];
			ADC_offset[1]+=ADC_ConvertedValue[1];
			tim_counter++;
		}
		else 
		{
			ADC_offset[0]/=5000;
			ADC_offset[1]/=5000;
			if ((ADC_offset[0]>1400 && ADC_offset[0]<1700)
			&&  (ADC_offset[1]>1400 && ADC_offset[1]<1700))
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
float current_stream[2];
float motor_current=0;
//u32 current_memory_index=0;
//float current_memory_sum=0;

void get_current(void)
{
	current_stream[0]=-ADC_ConvertedValue[0]+ADC_offset[0];
	current_stream[1]=-ADC_ConvertedValue[1]+ADC_offset[1];

	if (abs(current_stream[0])>abs(current_stream[1]))
		motor_current=current_stream[0];
	else
		motor_current=current_stream[1];
/*
	if (current_memory_index<9)
		current_memory_index++;
	else
		current_memory_index=0;

	current_memory_sum-=current_memory[current_memory_index];
	current_memory[current_memory_index]=motor_current;
	current_memory_sum+=current_memory[current_memory_index];
*/	
}

void current_circle(float p, float i, float d)
{
	I_err_curr=current_input-motor_current;

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

void vel_move(u8 mode)
{
	vel_err();

	if (mode==0)
	{

	 	vel_calculate_pwm(kp1,ki1,kd1,0);
	
	//	current_input=r_count;
	
		current_circle(ip,ii,id);
	}
	else
	{
		vel_calculate_pwm(kp11,ki11,kd11,1);
	}

}

//sub function
void vel_stop(u8 mode)
{  
	r_count=0;

	vel_err();

	if (mode==0)
	{
		vel_calculate_pwm(kp2,ki2,kd2,0);
	
		current_circle(ip,ii,id);
	}
	else
	{
		vel_calculate_pwm(kp21,ki21,kd21,1);
	}
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



float pwm_current=0;
float max_current=1000;
float r_cur=0;
void vel_pwm_current(u8 mode)
{
	if (mode==0)
	{
		if (pwm_current==0)
			pwm=0;
		else
		{
			if (pwm_current>0)
				r_cur=max_current;
			else
				r_cur=-max_current;
	
		    I_err_curr=r_cur-motor_current;
		
		    I_cal_P = ip * (I_err_curr - I_err_prev1);
			I_cal_I = ii * I_err_curr;
			I_cal_D = id * (I_err_curr + I_err_prev2 - 2*I_err_prev1);
		
			I_err_prev2 = I_err_prev1;
			I_err_prev1 = I_err_curr;
				
			I_PID = I_cal_P + I_cal_I + I_cal_D;
		
			pwm = pwm + I_PID;
		
			dir = (pwm>0)?1:0;
			if (pwm_current>0)
			{
				if(pwm >pwm_current)	pwm = pwm_current;
				if(pwm > MAX_PWM)	pwm = MAX_PWM;
				if(pwm < -MAX_PWM)   pwm = -MAX_PWM;
			}
			else 
			{
				if(pwm < pwm_current)   pwm = pwm_current;
				if(pwm > MAX_PWM)	pwm = MAX_PWM;
				if(pwm < -MAX_PWM)   pwm = -MAX_PWM;
			}
		}
	}
	else
	{
		pwm=pwm_current;
	}
}

// main function
void vel_n_pos(void)
{

    read_encoder();
	get_current();
	if (cali_done)
	{
		if (enable_bar==0)  
			vel_zero();
	
		else if(enable_bar==2)
			vel_stop(0);
		
		else if(enable_bar==1) 
			vel_move(0);
		
		else if(enable_bar==3)
			pos_move(0);
	
		else if(enable_bar==4)
			vel_pwm_current(0);

		else if(enable_bar==5) 
			vel_pwm();
		
	}
	else 
	{
		if(enable_bar==6) 
			ADC_calibration();

		else if (enable_bar==0)  
			vel_zero();
	
		else if(enable_bar==2)
			vel_stop(1);
		
		else if(enable_bar==1) 
			vel_move(1);
		
		else if(enable_bar==3)
			pos_move(1);
	
		else if(enable_bar==4)
			vel_pwm_current(1);

		else if(enable_bar==5) 
			vel_pwm();
	}
	set_pwm_to_motor();	   
}

// function	called by user
void motor_set_pwm(float user_pwm)
{
	enable_bar=5;

	set_pwm=user_pwm;
	set_dir = (user_pwm>=0)?1:0;
}

void motor_set_pwm_current(float user_pwm)
{
	enable_bar=4;

	pwm_current=user_pwm;
}

// function	called by user
void motor_set_speed(float speed)
{
    enable_bar=1;
    user_speed=speed;
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

void cali_user()
{
	enable_bar=6;
}

void set_acceleration(u16 value)
{
	increment=((float)(value))/1000;
}

// comm with main board for debugging
void increase_encoder(void)
{
    abs_encoder+=(s32)(d_count);
}





