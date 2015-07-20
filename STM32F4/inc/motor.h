/*
 * motor.h
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "encoder.h"
#include "Leds.h"

GPIO* const PWM_GPIO = &PB14;

GPIO* const DIR_A = &PB12;
GPIO* const DIR_B = &PB13;

TIMER* const PWM_TIM = &TIM12Ch1;

const int no_of_motor = 4;
const int control_freq = 40000;
enum dir
{
	ANTI_CKW = -1,
	DONT_CARE = 0,
	CKW = 1
};


enum motor_error {
	NULL_pointer = 0
};

namespace _motor {
	class motor : public encoder
	{
	public:
		motor(GPIO* const phaseA_gpio, GPIO* const phaseB_gpio, TIM_TypeDef* encoder_TIM, \
				GPIO* const motor_mag, GPIO* const motor_dirA, GPIO* const motor_dirB, TIMER* const motor_TIM);
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
		float get_current_vel();
		float get_pwm();
		bool is_open_loop();
		bool is_overspeed();
		const int MAX_PWM = SystemCoreClock / control_freq - 1;
		const float F1toF4_CORR = (MAX_PWM + 1) / 1800.0;
		int cal_error();

	private:
		void output_pwm(int pwm);
		void set_dir(dir direction);

		bool is_close_loop;
		int target_vel;
		float curr_vel;
		float acceleration;

		TIM_TypeDef* const motor_timer;
		static const int MAX_ACCEL = 1000;
		static const int MIN_ACCEL = 5;

		// Access

		// Output
		float curr_pwm;
		bool overspeed;
		GPIO* const dirA_gpio;
		GPIO* const dirB_gpio;

	};

}

using namespace _motor;

#endif /* MOTOR_H_ */
