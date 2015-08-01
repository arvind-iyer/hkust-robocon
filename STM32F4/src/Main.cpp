/*
 * Main.cpp
 *
 *  Created on: 2015-04-02
 *      Author: William
 */

#include <inttypes.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <Ticks.h>
#include <Leds.h>
#include <Usart.h>
#include <buzzer_song.h>
#include <can_protocol.h>
#include "stm32f4xx_can.h"
#include <can_motor.h>
#include <stm32f4xx_it.h>
#include <cmath>
#include "encoder.h"
#include <stdio.h>
#include "motor.h"
#include "gpio.h"


static int ticks_img = 655;

void HardFault_Handler(void)	// Indicate any seg fault.
{
	Leds Signal6(LED_6, LED_ON);
	while (true) {
		if (ticks_img != Ticks::getInstance()->getTicks()) {
			ticks_img = Ticks::getInstance()->getTicks();
			if (Ticks::getInstance()->getFullTicks() % 2000 == 0) {
				FAIL_MUSIC;
				Signal6.led_toggle();
			}
			buzzer_check();
		}
	}
}

void UsageFault_Handler(void) // Any Division by 0 or relevant runtime error
{
	Leds Signal12(LED_12, LED_ON);
	while (true) {
		if (ticks_img != Ticks::getInstance()->getTicks()) {
			ticks_img = Ticks::getInstance()->getTicks();
			if (Ticks::getInstance()->getFullTicks() % 2000 == 0) {
				FAIL_MUSIC;
				Signal12.led_toggle();
			}
			buzzer_check();
		}
	}
}

void FPU_IRQHandler()
{
	Leds Signal12(LED_12, LED_ON);
}

int main()
{
	Ticks ticks;
//	Usart uart1(USART1, 115200);
//	uart1.setPrintUsart(USART1);
	buzzer_init();
	motor motor_up(encoder_U_A, encoder_U_B, TIM4, motor_U_pwm, motor_U_dir, motor_U_TIM);
	motor motor_down(encoder_D_A, encoder_D_B, TIM4, motor_D_pwm, motor_D_dir, motor_D_TIM);
	motor motor_right(encoder_R_A, encoder_R_B, TIM4, motor_R_pwm, motor_R_dir, motor_R_TIM);
	motor motor_left(encoder_L_A, encoder_L_B, TIM4, motor_L_pwm, motor_L_dir, motor_L_TIM);
	can_init();
	can_rx_init();
	can_motor_init();
	Leds Signal1(LED_1, LED_ON), Signal2(LED_2, LED_OFF);
	buzzer_play_song(START_UP, 125, 0);
	while (true) {
		static int last_speed = 0;
		if (ticks_img != ticks.getTicks()) {
			ticks_img = ticks.getTicks();
			buzzer_check();
			motor_left.encoder::refresh();
//			if (last_speed != motor_left.get_change_of_encoder()) {
//				last_speed = motor_left.get_change_of_encoder();
//				motor_left.send_feedback();
//			}
			if (ticks_img % (1000 / motor_left.get_accel()) == 0) {
				motor_left.refresh();
			}

			if (ticks_img % 100 == 0) {
				motor_up.send_feedback();
				motor_down.send_feedback();
				motor_right.send_feedback();
				motor_left.send_feedback();
			}

			if (ticks_img % 1000 == 0) {

				if (!motor_left.is_encoder_work) {

					Signal1.led_control(ticks.getSeconds() % 2 ? LED_ON : LED_OFF);
					Signal2.led_control(ticks.getSeconds() % 2 ? LED_ON : LED_OFF);
					FAIL_MUSIC;
				} else if (motor_left.is_overspeed()) {
					buzzer_play_song(SUCCESSFUL_SOUND,1000,0);
					Signal1.led_control(LED_ON);
					Signal2.led_control(LED_ON);
				} else {
					Signal1.led_control(ticks.getSeconds() % 2 ? LED_ON : LED_OFF);
					Signal2.led_control(ticks.getSeconds() % 2 ? LED_OFF : LED_ON);

					buzzer_off();
				}
			}
		}
	}
}
