/*
 * motor.cpp
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */

#include "motor.h"
#include <cmath>
#define increment_PID

#ifndef increment_PID
static float ideal_pos = 0;
#endif

static motor* motor_x[no_of_motor] = {0};

motor::motor(GPIO* const phaseA_gpio, GPIO* const phaseB_gpio, TIM_TypeDef* encoder_TIM, GPIO* const motor_mag, GPIO* const motor_dir, TIMER* const motor_TIM) :
encoder(phaseA_gpio, phaseB_gpio, encoder_TIM), is_close_loop(false), overspeed(false),
curr_pwm(0), target_vel(0), curr_vel(0), acceleration(100), motor_timer(motor_TIM->TIMx), dir_gpio(motor_dir)
{

	motor_mag->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	motor_dir->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	GPIO_PinAFConfig(motor_mag->gpio, motor_mag->get_pin_source(), GPIO_AF_TIM11);
  timer_init(motor_TIM->TIMx, 0, TIM_CounterMode_Up, MAX_PWM, TIM_CKD_DIV1);
  pwm_timer_init(*motor_TIM, TIM_OCMode_PWM1, TIM_OutputState_Enable, MAX_PWM, TIM_OCPolarity_High);
  dir_gpio->gpio_write(Bit_RESET);
  if (this_id < no_of_motor) {
  	motor_x[this_id] = this;
  }
}

void motor::set_accel(unsigned int accel)
{
	acceleration = (accel > MAX_ACCEL ? MAX_ACCEL : (accel < MIN_ACCEL ? MIN_ACCEL : accel));
}

void motor::set_target_vel(int vel)
{
	target_vel = vel;
	is_close_loop = true;
}

void motor::set_pwm(int pwm)
{
	curr_pwm = pwm;
	is_close_loop = false;
}

unsigned int motor::get_accel()
{
	return acceleration;
}

int motor::get_target_vel()
{
	return target_vel;
}

int motor::get_current_vel()
{
	return curr_vel;
}

int motor::get_pwm()
{
	return curr_pwm;
}

void motor::lock()
{
	target_vel = curr_vel = 0;
	is_close_loop = true;
}

void motor::refresh()
{
	if (curr_vel > target_vel) {
		--curr_vel;
	} else if  (curr_vel < target_vel) {
		++curr_vel;
	}
}

int motor::cal_error()
{
#ifdef increment_PID
	return curr_vel - get_change_of_encoder();
#else
	return get_encoder_value() - ideal_pos;
#endif
}


void motor::pid_control(float p, float i, float d)
{
	const int TIMEOUT = 100;

	static float prev_error = 0;
	static int encoder_fail_time_out = 0;
#ifdef increment_PID
	static float last_prev_error = 0;
	float PID = p*(cal_error() - prev_error) + i*cal_error() + d*(cal_error() + last_prev_error - prev_error*2);
#else
	static float acumulate_error = 0;
	float PID = p*(cal_error()) + i*acumulate_error + d*(cal_error() - prev_error);
	ideal_pos += curr_vel/5.0;
#endif
	if (is_close_loop && is_encoder_work) {
		if (std::abs(curr_vel) > 0 && get_change_of_encoder() == 0) {
			++encoder_fail_time_out;
			is_encoder_work = (encoder_fail_time_out > TIMEOUT ? false : true);
		} else {
			encoder_fail_time_out = 0;
		}
#ifdef increment_PID
		curr_pwm += PID;
#else
		curr_pwm = PID;
#endif
	}
	else {
    // Disable PID control if encoder is not work, no pwm to motor and warning light.
    if (is_close_loop && !is_encoder_work) {
      curr_pwm = 0;
    }

    // No PID control velocity, velocity will be the differentiate of encoder value.
    if (is_encoder_work) {
      target_vel = curr_vel = get_change_of_encoder();
    }

    // Enable PID control again if the encoder is working
    if (!is_encoder_work && get_change_of_encoder() != 0) {
      encoder_fail_time_out = 0;
      is_encoder_work = true;
    }
	}
#ifdef increment_PID
	last_prev_error = prev_error;
#else
	acumulate_error += cal_error();
#endif
	prev_error = cal_error();
	output_pwm(curr_pwm);
}

void motor::set_dir(dir direction)
{
	if (direction == ANTI_CKW) {
		dir_gpio->gpio_write(Bit_RESET);
	} else if (direction == CKW) {
		dir_gpio->gpio_write(Bit_SET);
	}

}

bool motor::is_overspeed()
{
	return overspeed;
}

void motor::output_pwm(int pwm)
{
	if (pwm > MAX_PWM) {
		pwm = MAX_PWM;
		overspeed = true;
	} else if (pwm < -MAX_PWM) {
		pwm = -MAX_PWM;
		overspeed = true;
	} else {
		overspeed = false;
	}

	TIM_SetCompare1(motor_timer, uint32_t(MAX_PWM - std::abs(pwm)));

	set_dir(static_cast<int>(pwm) > 0 ? CKW : (static_cast<int>(pwm) < 0 ? ANTI_CKW : DONT_CARE));
}

motor* motor::get_instance(unsigned int access_id) throw(motor_error)
{
	if (access_id > no_of_motor || motor_x[access_id] == 0) {
		throw (motor_error)NULL_pointer;
	}
	return motor_x[access_id];
}
