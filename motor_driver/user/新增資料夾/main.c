#include "main.h"


#define EDROID 1
#define MCS 0
#define REAL 2

//communication
extern u16 cmd;
extern u8 cmd_flag, data_cnt;
extern u8 rx_buffer[];

//admin
extern u8 start; //1 or 0
float conv_start;
extern u8 mode;	 //1, 2, 3, 4, 5
float conv_mode;

//velocity mode
extern float r_count;
extern float d_count;
extern float kp1; //pid for moving
extern float ki1;
extern float kd1;
extern float kp2; //pid for break off
extern float ki2;
extern float kd2;
extern float ip;
extern float ii;
extern float id;
extern float pwm;
extern float pwm_init;
extern s8 dir;
//extern float max_count_vel; //-1 means not set

//position mode
extern float kp_pos;
extern float ki_pos;
extern float kd_pos; 
extern float count_dest;
extern float count_total;
extern float max_acc;	   //max acceleration

extern float max_pwm;    //max speed

//just ticks
extern u8 ticks;

//0 means mcs debugger, 1 means edroid debugger, 2 means simulator or real use
u8 debug_mode = REAL;

extern u8 port_index;
extern u8 enable_bar;

//testing
u8 test[4];
extern u8 tx_data_size;
extern u8 tx_buffer[];
extern u8 tx_buffer_token;
extern float delta;
extern float histogram[];
extern float ADC_offset;

int main(void)
{
	system_init(5000);
//	system_para_init();
//	system_pwm_enable();
//	init_memory();
	CAN_Configuration();

		while(1);
}

