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
extern float ADC_offset[];
extern u8 cali_done;
extern float d_cal;
extern u8 reach_token;
extern vu16  ADC_ConvertedValue[];
u32 delay_i=0;
u16 ticks_img = 0;

u8 index = 0;
float integrated_dcount = 0;

int main(void)
{
	system_init(5000);
	system_para_init();
	system_pwm_enable();
	init_memory();
	ticks_init();
	CAN_Configuration();
  
	uart_init(COM1, 115200);
	
	while ((ADC_ConvertedValue[0]<1400 || ADC_ConvertedValue[0]>1700)
	     ||(ADC_ConvertedValue[1]<1400 || ADC_ConvertedValue[1]>1700)){
				 GPIO_WriteBit(GPIOC,GPIO_Pin_1,1);
				 GPIO_WriteBit(GPIOC,GPIO_Pin_0,1);
				 
				 if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 100 == 0)
				Send_Encoder((s32)(d_cal));
			
			if(ticks_img == 0) {
				 uart_tx(COM1, "Seconds: %d", get_seconds());
			 }
						 
			}
		

		 }

	for(delay_i = 0; delay_i < 1000; delay_i++);

	cali_user();



	while (!cali_done)
	{
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 5 == 0)
			{
			//	debugger_msg();	
				Device1_TX();
			}
		}
	}

   
				 GPIO_WriteBit(GPIOC,GPIO_Pin_1,0);
				 GPIO_WriteBit(GPIOC,GPIO_Pin_0,0);
	
	motor_zero();
	for(index = 0; index < 10; index++)	
		Calibration_Done();
//	motor_set_position(280000,300);
	
	while(1)
	{
	  
		if (ticks_img != get_ticks()) 
		{
			ticks_img = get_ticks();
		
			if(ticks_img % 5 == 0)
			{
//				debugger_msg();	
//	            Motor_Return_Encoder(d_count);
//				debug_data1((s32) count_dest);
//                debug_data2((s32) count_total);
//                debug_data3((s32) d_cal);
//                debug_data4((s32) enable_bar);
 
//				if (reach_token)
//					motor_set_position(280000,300);	
				Send_Encoder((s32)(d_cal));
			}
		}

/*	 	
	
		if (cmd_flag)
		{
			//GUI shake command. Compatible with edroid debugger and mcs debugger
			if (cmd == shake_cmd)
			{
			//	mode = CALIBRATION;
				cali_user();
				system_shake_hand(shake_cmd);
			}
			else if (cmd == shake2_cmd && data_cnt == 0)
			{
				system_shake_hand(shake2_cmd);	
			}
			//----------------------------------------------------------------------//
			if(debug_mode == REAL)
			{
				//admin
				if(cmd == enable_cmd) start = 1;
				else if(cmd == disable_cmd) start = 0;

				//parameters setting in velocity mode
				else if(cmd == vel_init_cmd && data_cnt == 4 )
				{
				 	u8 tmp_pwm[4];
					
					tmp_pwm[0] = rx_buffer[0];
					tmp_pwm[1] = rx_buffer[1];
					tmp_pwm[2] = rx_buffer[2];
					tmp_pwm[3] = rx_buffer[3];
					
					pwm_init = *((float*)tmp_pwm);
				}
				else if(cmd == vel_pid1_cmd && data_cnt == 12)
				{
				 	u8 tmp_kp[4];
					u8 tmp_ki[4];
					u8 tmp_kd[4];

					tmp_kp[0] = rx_buffer[0];
					tmp_kp[1] = rx_buffer[1];
					tmp_kp[2] = rx_buffer[2];
					tmp_kp[3] = rx_buffer[3];
	
					tmp_ki[0] = rx_buffer[4];
					tmp_ki[1] = rx_buffer[5];
					tmp_ki[2] = rx_buffer[6];
					tmp_ki[3] = rx_buffer[7];
	
					tmp_kd[0] = rx_buffer[8];
					tmp_kd[1] = rx_buffer[9];
					tmp_kd[2] = rx_buffer[10];
					tmp_kd[3] = rx_buffer[11];
					
					kp1 = *((float*)tmp_kp);
					ki1 = *((float*)tmp_ki);
					kd1 = *((float*)tmp_kd);
				}
				else if(cmd == vel_pid2_cmd && data_cnt == 12)
				{
				 	u8 tmp_kp[4];
					u8 tmp_ki[4];
					u8 tmp_kd[4];

					tmp_kp[0] = rx_buffer[0];
					tmp_kp[1] = rx_buffer[1];
					tmp_kp[2] = rx_buffer[2];
					tmp_kp[3] = rx_buffer[3];
	
					tmp_ki[0] = rx_buffer[4];
					tmp_ki[1] = rx_buffer[5];
					tmp_ki[2] = rx_buffer[6];
					tmp_ki[3] = rx_buffer[7];
	
					tmp_kd[0] = rx_buffer[8];
					tmp_kd[1] = rx_buffer[9];
					tmp_kd[2] = rx_buffer[10];
					tmp_kd[3] = rx_buffer[11];
				
					kp2 = *((float*)tmp_kp);
					ki2 = *((float*)tmp_ki);
					kd2 = *((float*)tmp_kd);
				}
				else if(cmd == vel_max_cmd && data_cnt == 4)
				{
				 	u8 tmp_max[4];

					tmp_max[0] = rx_buffer[0];
					tmp_max[1] = rx_buffer[1];
					tmp_max[2] = rx_buffer[2];
					tmp_max[3] = rx_buffer[3];
					
					max_pwm = *((float*)tmp_max);
				}

				//action in velocity mode
				else if(cmd == vel_move_cmd && data_cnt == 4)
				{
				 	u8 tmp_move[4];

					tmp_move[0] = rx_buffer[0];
					tmp_move[1] = rx_buffer[1];
					tmp_move[2] = rx_buffer[2];
					tmp_move[3] = rx_buffer[3];
			
					r_count = *((float*)tmp_move); 
					mode = VEL_MOVE;
				}
				else if(cmd == vel_stop_cmd )
				{
					mode = VEL_STOP;
				}
				else if(cmd == vel_zero_cmd && data_cnt == 0)
				{
				 	mode = VEL_ZERO;
				}	   

				//parameters setting in position mode
				else if(cmd == pos_pid_cmd && data_cnt == 12)
				{
				 	u8 tmp_kp[4];
					u8 tmp_ki[4];
					u8 tmp_kd[4];

					tmp_kp[0] = rx_buffer[0];
					tmp_kp[1] = rx_buffer[1];
					tmp_kp[2] = rx_buffer[2];
					tmp_kp[3] = rx_buffer[3];
	
					tmp_ki[0] = rx_buffer[4];
					tmp_ki[1] = rx_buffer[5];
					tmp_ki[2] = rx_buffer[6];
					tmp_ki[3] = rx_buffer[7];
	
					tmp_kd[0] = rx_buffer[8];
					tmp_kd[1] = rx_buffer[9];
					tmp_kd[2] = rx_buffer[10];
					tmp_kd[3] = rx_buffer[11];
				
					ip = *((float*)tmp_kp);
					ii = *((float*)tmp_ki);
					id = *((float*)tmp_kd);	  
				}
				else if(cmd == pos_max_cmd && data_cnt == 8)  //max_acc followed by max_vel
				{
				 	u8 tmp_acc[4];

					tmp_acc[0] = rx_buffer[0];
					tmp_acc[1] = rx_buffer[1];
					tmp_acc[2] = rx_buffer[2];
					tmp_acc[3] = rx_buffer[3];
					
					max_acc = *((float*)tmp_acc);
	
				}
				
				//action in position mode
				else if(cmd == pos_move_cmd && data_cnt == 4)//need to set home first
				{
			 
				 	u8 tmp_move[4];

					tmp_move[0] = rx_buffer[0];
					tmp_move[1] = rx_buffer[1];
					tmp_move[2] = rx_buffer[2];
					tmp_move[3] = rx_buffer[3];
					
					count_dest = *((float*)tmp_move); //need unit conversion!!!!!!!!!
					mode = POS_MOVE;
				   
					motor_set_position(count_dest,300);
				}
				else if(cmd == pos_lock_cmd && data_cnt == 0)//need to set home first
				{
					mode = POS_LOCK;
				}
				else if(cmd == pos_home_cmd && data_cnt == 0)
				{
					pos_set_home();
				}


				
				//send back data
				else if(cmd == get_vel_actual_cmd)
				{
					send_float(get_vel_actual_cmd, &d_count);
				}
				else if(cmd == get_vel_desire_cmd)
				{
					send_float(get_vel_desire_cmd, &r_count);
				}
				else if(cmd == get_pos_actual_cmd)
				{
					send_float(get_pos_actual_cmd, &count_total);
				}
				else if(cmd == get_pos_desire_cmd)
				{
					send_float(get_pos_desire_cmd, &count_dest);
				}
				else if(cmd == get_vel_pid1_cmd)
				{
                    send_3_floats(get_vel_pid1_cmd, &kp1, &ki1, &kd1);
				}
				else if(cmd == get_vel_pid2_cmd)
				{
					send_3_floats(get_vel_pid2_cmd, &kp2, &ki2, &kd2);
				}
				else if(cmd == get_pos_pid_cmd)
				{
					send_3_floats(get_pos_pid_cmd, &ip, &ii, &id);
				}
				else if(cmd == get_max_vel_cmd)
				{
					send_float(get_max_vel_cmd, &histogram[3]);
				}
				else if(cmd == get_max_acc_cmd)
				{
					send_float(get_max_acc_cmd, &histogram[4]);
				}
			
				else if(cmd == get_vel_init_cmd)
				{
					send_float(get_vel_init_cmd, &histogram[5]);
				}
				else if(cmd == get_mode_cmd)
				{
					conv_mode = enable_bar;
					send_float(get_mode_cmd, &ADC_offset[0]);
				}
				else if(cmd == get_enable_cmd)
				{
					if(start == 1) conv_start = 1;
					else 		   conv_start = 0;
					send_float(get_enable_cmd, &ADC_offset[1]);
				} 
			}	   

			
			cmd_flag = 0;
		} 
	  	  */
	}	  
}

