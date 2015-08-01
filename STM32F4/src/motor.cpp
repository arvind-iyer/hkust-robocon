/*
 * motor.cpp
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */

#include "motor.h"
#include <cmath>

static motor* m_MOTOR;

motor::motor(GPIO* const phaseA_gpio, GPIO* const phaseB_gpio, TIMER* const encoder_TIM, GPIO* const motor_mag, GPIO* const motor_dirA, GPIO* const motor_dirB, TIMER* const motor_TIM) :
encoder(phaseA_gpio, phaseB_gpio, encoder_TIM), is_close_loop(false), overspeed(false),
curr_pwm(0), target_vel(0), curr_vel(0), acceleration(100.0), motor_timer(motor_TIM),
dirA_gpio(motor_dirA), dirB_gpio(motor_dirB)
{
	// GPIO init
	motor_mag->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	motor_dirA->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	motor_dirB->gpio_init(GPIO_Speed_100MHz, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	// AF init
	motor_TIM->AF_init(motor_mag);
  timer_init(motor_TIM->TIMx, 0, TIM_CounterMode_Up, MAX_PWM, TIM_CKD_DIV1);
  pwm_timer_init(*motor_TIM, TIM_OCMode_PWM1, TIM_OutputState_Enable, MAX_PWM, TIM_OCPolarity_High);
  dirA_gpio->gpio_write(Bit_RESET);
  dirB_gpio->gpio_write(Bit_SET);
  m_MOTOR = this;
}

void motor::set_accel(unsigned int accel)
{
	acceleration = static_cast<float>(accel > MAX_ACCEL ? MAX_ACCEL : (accel < MIN_ACCEL ? MIN_ACCEL : accel));
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

float motor::get_current_vel()
{
	return curr_vel;
}

float motor::get_pwm()
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
	float accel_per_ms = acceleration / 1000.0;
	if (abs(curr_vel - target_vel) > accel_per_ms) {
		int dir = (target_vel - curr_vel) / abs(curr_vel - target_vel);
		curr_vel += dir * accel_per_ms;
	} else {
		curr_vel = target_vel;
	}
}

int motor::cal_error()
{
	return std::floor(curr_vel) - get_change_of_encoder();
}


void motor::pid_control(float p, float i, float d)
{
	const int TIMEOUT = 100;

	static float prev_error = 0;
	static int encoder_fail_time_out = 0;
	static float last_prev_error = 0;
	float PID = p*(cal_error() - prev_error) + i*cal_error() + d*(cal_error() + last_prev_error - prev_error*2);
	if (is_close_loop && is_encoder_work) {
		if (std::abs(std::floor(curr_vel)) > 0 && get_change_of_encoder() == 0) {
			++encoder_fail_time_out;
			is_encoder_work = (encoder_fail_time_out > TIMEOUT ? false : true);
		} else {
			encoder_fail_time_out = 0;
		}
		curr_pwm += PID;
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
	last_prev_error = prev_error;
	prev_error = cal_error();
	output_pwm(std::floor(curr_pwm) * F1toF4_CORR);
}

void motor::set_dir(dir direction)
{
	if (direction == ANTI_CKW) {
		dirA_gpio->gpio_write(Bit_RESET);
		dirB_gpio->gpio_write(Bit_SET);
	} else if (direction == CKW) {
		dirA_gpio->gpio_write(Bit_SET);
		dirB_gpio->gpio_write(Bit_RESET);
	}

}

bool motor::get_close_loop_state()
{
	return is_close_loop;
}

bool motor::get_overspeed_state()
{
	return overspeed;
}

void motor::output_pwm(int pwm)
{

	switch (static_cast<int>(std::abs(pwm) * F1toF4_CORR)) {
		case 0 ... 419:
			speed_indicator_on(0);
			break;
		case 420 ... 1259:
			speed_indicator_on(1);
			break;
		case 1260 ... 2099:
			speed_indicator_on(2);
			break;
		case 2100 ... 2939:
			speed_indicator_on(3);
			break;
		case 2940 ... 3779:
			speed_indicator_on(4);
			break;
		case 3780 ... 4199:
			speed_indicator_on(5);
			break;
		default:
			overspeed = true;
			speed_indicator_on(6);
			pwm = MAX_PWM * (pwm / std::abs(pwm));
	}

	motor_timer->SetCompare(uint32_t(MAX_PWM - std::abs(pwm)));

	set_dir(static_cast<int>(pwm) > 0 ? CKW : (static_cast<int>(pwm) < 0 ? ANTI_CKW : DONT_CARE));
}

motor* motor::get_instance() throw(motor_error)
{
	if (m_MOTOR == 0) {
		throw (motor_error)NULL_pointer;
	}
	return m_MOTOR;
}
