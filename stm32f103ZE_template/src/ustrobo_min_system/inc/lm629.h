#ifndef __LM629_H
#define __LM629_H
   
#include <inttypes.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "delay.h"

#define MENU_ADD_LM629_TEST menu_add(3, "LM629 Test", lm629_test);menu_add(4, "Encoder Test", encoder_test)

/*

	lm629_set_filter	0      0      30       0    1024    2048 # driving L
	lm629_set_filter	1      0      30       0    1024    2048 # driving R

*/
	 
/******************************************************************************
  Declaration
******************************************************************************/
// Wheels definition
#define LT_WHEEL 0
#define RT_WHEEL 1

// Flag to monitor trajectory completion
#define LM629_TRAJ_COMPLETION 0x08
#define LM629_POSITION_ERROR 0x04
#define LM629_BREAK_POINT 0x02

// LM629 Command Addresses
#define LM629_RESET    0x00 // Reset LM629
#define LM629_DFH      0x02 // Define Home
#define LM629_SIP      0x03 // Set Index Position
#define LM629_LPEI     0x1B // Interrupt on Error
#define LM629_LPES     0x1A // Stop on Error
#define LM629_SBPA     0x20 // Set Breakpoint, Absolute
#define LM629_SBPR     0x21 // Set Breakpoint, Relative
#define LM629_MSKI     0x1C // Mask Interrupts
#define LM629_RSTI     0x1D // Reset Interrupts
#define LM629_LFIL     0x1E // Load Filter Parameters
#define LM629_UDF      0x04 // Update Filter
#define LM629_LTRJ     0x1F // Load Trajectory
#define LM629_STT      0x01 // Start Motion
#define LM629_RDSIGS   0x0C // Read Signals Register
#define LM629_RDIP     0x09 // Read Index Position
#define LM629_RDDP     0x08 // Read Desired Position
#define LM629_RDRP     0x0A // Real Real Position
#define LM629_RDDV     0x07 // Read Desired Velocity
#define LM629_RDRV     0x0B // Read Real Velocity
#define LM629_RDSUM    0x0D // Read Integration Sum

#define cmdP GPIOD
#define RD GPIO_Pin_3		//port D
#define WR GPIO_Pin_4		//port D
#define PS GPIO_Pin_5		//port D
#define RST GPIO_Pin_6		//port D

#define CS0 GPIO_Pin_1		//port B
#define CS1 GPIO_Pin_0		//port B
#define CS2 GPIO_Pin_5		//port C
#define CS3 GPIO_Pin_4		//port C

#define dataP GPIOG
#define D0 GPIO_Pin_8		//port G
#define D1 GPIO_Pin_9		//port G
#define D2 GPIO_Pin_10		//port G
#define D3 GPIO_Pin_11		//port G
#define D4 GPIO_Pin_12		//port G
#define D5 GPIO_Pin_13		//port G
#define D6 GPIO_Pin_14		//port G
#define D7 GPIO_Pin_15		//port G

//Reset Trial
#define ResetTrial 10

#define TrajCompleteStatus 2
#define PositionError 5
#define BreakPoint 6
//vel=4*MOTOR_CPR*65536*vel*MOTOR_RATIO*2048/(LM629_CLK*1000000);  

#define PI	3
#define MAX_CHIP 10

#define	m2count(M,CPR,GEAR,DIAMETER) 4*CPR*GEAR*M/(DIAMTER*PI)
#define	ms2count(MS,CPR,GEAR,DIAMETER) 4*65536*2048/6e6*CPR*MS/(DIAMTER*PI)*GEAR
#define ms22count(MS2,CPR,GEAR,DIAMETER) 4*65536*2048*2048/6e6/6e6*CPR*MS2/(DIAMETER*PI)*GEAR

#define count2m(C,CPR,GEAR,DIAMTER) C*DIAMTER*PI/(4*CPR*GEAR)
#define count2ms(C,CPR,GEAR,DIAMTER) C*DIAMTER*PI/(4*65546*2048*6e6*CPR*GEAR)

/******************************************************************************
  Variable Declaration
******************************************************************************/
// 2Hz and 64Hz flag
extern u16 lm629_flag;
extern u16 lm629_chip_available;


extern u16 lm629_filter[MAX_CHIP][4];
extern u32 lm629_acc[MAX_CHIP];
extern u16 lm629_pe[MAX_CHIP];	
extern u8 dataP_dir;

//set bit
void set_dataP_dir(u8);	//parametre=1-->input
u8 read_dataP(void);
void write_dataP(u8);	

void SelectChip(u8 chip);
u8 ReadStatus(void);

void lm629_init (void);	  							//init
void lm629_set_filter (uc8 chip,uc16 kp,uc16 ki,uc16 kd,uc16 il);  	   //set parameters= KP , KI , KD , IL
void lm629_acceleration (uc8 chip,uc32 acc);		//set acceleration
void lm629_define_home (uc8 chip);					//define encoder value=0

void lm629_zero_drive (uc8 chip);					//set zero driven ==> no driving force
void lm629_stop_abruptly (uc8 chip);				//set stop abruptly ==>	lock the motor
void lm629_stop_smoothly(uc8 chip);					//set stop smoothly ==> stop with auto deceleration
void lm629_disable_output(void);   				   	//set all motors to be zero driven
void lm629_enable_output(void);	
void lm629_reset_pe(uc8 chip);

void lm629_velocity_start (uc8 chip,s32 vel);		//set velocity
void lm629_position_start (uc8 chip,uc32 vel,sc32 pos);

s32 lm629_read_pos (uc8 chip);				   		//read encoder
s32 lm629_read_pos_home (uc8 chip);
s32 lm629_read_vel(uc8 chip);
u8 lm629_pos_done(uc8 chip);
u8 lm629_is_pe(uc8 chip);
u8 lm629_bp_reach(uc8 chip);

s32 lm629_read_vel(uc8 chip);
s32 lm629_read_des_vel(uc8 chip);
s32 lm629_read_des_pos (uc8 chip);

void lm629_read_pos_twin (s32 * pos0,s32 * pos1);
void lm629_read_pos_twin_home (s32 * pos0,s32 * pos1);
 
void lm629_velocity_twin_start (s32 vel1, s32 vel2);  	  
void lm629_position_twin_start (uc32 vel1,uc32 vel2, s32 pos);

void lm629_abs_position_start (uc8 chip,uc32 vel,sc32 pos);
void lm629_rel_position_start (uc8 chip,uc32 vel,sc32 pos);

void lm629_set_pe_report (uc8 chip, uc16 pe);
void lm629_set_pe_stop (uc8 chip, uc16 pe);

void lm629_set_abs_bp (uc8 chip,sc32 bp);
void lm629_set_rel_bp (uc8 chip,sc32 bp);		 

void lm629_abs_pos_start_done(uc8 chip, uc32 vel, sc32 pos);
void lm629_abs_pos_start_done_twin(uc8 motion, uc32 vel, sc32 pos);

#endif		/* __LM629_H */
