#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include "stm32f10x.h"
//#include "stm32f10x_it.h"

//user source code
#include "motion.h"
#include "system.h"
#include "pid_vel.h"
#include "pid_pos.h"
#include "debugger.h"
#include "Robocon_CAN.h"
#include "Robocon_CANProtocol.h"
#include "ticks.h"
#include "uart.h"
#define stop_byte1	0xfe
#define stop_byte2	0xff
#define shake_cmd	0x01
#define shake2_cmd	0x02
#define hiden_cmd	0x05

/*
#define test_auto_cmd  	0x11
#define test_pos_cmd  	0x12
#define test_vel_cmd 	0x13
#define test_start_cmd  0x14
#define test_stop_cmd	0x15
#define test_pid_cmd	0x16
#define test_init_cmd	0x17
*/
//-------------------------------//
#define enable_cmd			0x07
#define disable_cmd			0x08

#define vel_init_cmd		0x10
#define vel_pid1_cmd		0x11
#define vel_pid2_cmd		0x12
#define	vel_max_cmd			0x13
#define vel_move_cmd		0x15
#define vel_stop_cmd		0x16
#define vel_zero_cmd		0x17

#define pos_pid_cmd			0x20
#define pos_max_cmd			0x21
#define pos_move_cmd		0x23
#define pos_lock_cmd		0x24
#define pos_home_cmd		0x25

#define get_vel_actual_cmd	 0x40
#define get_vel_desire_cmd	 0x41
#define get_pos_actual_cmd	 0x42
#define get_pos_desire_cmd	 0x43
#define get_vel_pid1_cmd	 0x44
#define get_vel_pid2_cmd	 0x45
#define get_pos_pid_cmd		 0x46
#define get_max_vel_cmd		 0x47
#define get_max_acc_cmd		 0x48
#define get_vel_init_cmd	 0x49
#define get_mode_cmd	 	 0x4A
#define get_enable_cmd		 0x4B
#define get_min_acc_cmd		 0x4C

#define send_float32_cmd	 0x31

//---------------------------------//
/*
#define mon_kp_byte  0x20
#define mon_ki_byte  0x21
#define mon_kd_byte  0x22
#define mon_start_byte  0x23
#define mon_mode_byte  0x24
#define mon_pwm_byte  0x25
#define mon_dir_byte  0x26 
#define mon_verr_byte  0x27
#define mon_perr_byte  0x28
#define mon_pid_byte  0x29
#define mon_vel_byte  0x30
#define mon_pos_byte  0x31
*/
#define rx_buf_size 50
#define full_pwm	80
#define half_pwm	70

#define VEL_MOVE 1
#define VEL_STOP 2
#define VEL_ZERO 3
#define POS_MOVE 4
#define POS_LOCK 5
#define CALIBRATION 6

#endif

