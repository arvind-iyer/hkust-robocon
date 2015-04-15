/*
 * motor.h
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "encoder.h"

enum dir
{
	ANTI_CKW = -1,
	DONT_CARE = 0,
	CKW = 1
};

GPIO* const MOTOR_MAG = &PB9;
GPIO* const MOTOR_DIR = &PB5;

const int no_of_motor = 4;

enum motor_error {
	NULL_pointer = 0
};

namespace _motor {
	class motor : public encoder
	{
	public:
		motor(GPIO* const phaseA_gpio, GPIO* const phaseB_gpio, TIM_TypeDef* encoder_TIM, \
				GPIO* const motor_mag, GPIO* const motor_dir, TIMER* const motor_TIM);
		static motor* get_instance(unsigned int access_id) throw(motor_error);
	// Set function
		void set_target_vel(int vel);
		void set_pwm(int pwm);
		void set_accel(unsigned int accel);
		void lock();

		void pid_control(float p, float i, float d);
		void refresh();
		// Get function
		unsigned int get_accel();
		int get_target_vel();
		int get_current_vel();
		int get_pwm();

		bool is_overspeed();

	private:
		int cal_error();
		void output_pwm(int pwm);
		void set_dir(dir direction);

		bool is_close_loop;
		int target_vel;
		int curr_vel;
		unsigned int acceleration;

		TIM_TypeDef* const motor_timer;
		const int MAX_PWM = SystemCoreClock / 2 / 10000 - 1;
		const int MAX_ACCEL = 1000;
		const int MIN_ACCEL = 5;

		// Access

		// Output
		float curr_pwm;
		bool overspeed;
		GPIO* const dir_gpio;

	};

}

using namespace _motor;

#endif /* MOTOR_H_ */
