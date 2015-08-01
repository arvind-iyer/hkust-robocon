/*
 * encoder.cpp
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */

#include "encoder.h"
#include "Leds.h"

static int current_id = 0;

encoder::encoder(GPIO* const phaseA, GPIO* const phaseB, TIM_TypeDef* const TIMx) :
vel(0),	m_TIM(TIMx), encoder_value(0), flowing_flag(0), is_encoder_work(true), this_id(current_id)
{
	++current_id;
	phaseA->gpio_init(GPIO_Speed_50MHz, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	phaseB->gpio_init(GPIO_Speed_50MHz, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
	GPIO_PinAFConfig(phaseA->gpio, phaseA->get_pin_source(), GPIO_AF_TIM3);
	GPIO_PinAFConfig(phaseB->gpio, phaseB->get_pin_source(), GPIO_AF_TIM3);
  timer_init(m_TIM, 0, TIM_CounterMode_Up, 65535, TIM_CKD_DIV1);
  encoder_timer_init(m_TIM, SOURCE_ENCODER_INIT_VAL);
}

int encoder::get_encoder_value(void)
{
	return encoder_value;
}

int encoder::get_change_of_encoder(void)
{
	return vel;
}

void encoder::refresh(void)
{
	static int last_encoder_value = encoder_value;
	volatile s32 source_encoder = TIM_GetCounter(m_TIM);	// value directly from timer count
	if (source_encoder > 62768) {
		// Treat as underflow, add 30000 to timer counter.
		TIM_SetCounter(m_TIM, source_encoder - 30000);
		++flowing_flag;
	} else if (source_encoder < 2768) {
		// Treat as underflow, add 30000 to timer counter.
		TIM_SetCounter(m_TIM, source_encoder + 30000);
		--flowing_flag;
  }
	encoder_value = flowing_flag * 30000 + TIM_GetCounter(m_TIM) - SOURCE_ENCODER_INIT_VAL;
	vel = last_encoder_value - encoder_value;
	last_encoder_value = encoder_value;
}

void encoder::send_feedback(void)
{
	CAN_MESSAGE msg;
  msg.id = CAN_MOTOR_BASE + this_id;
  msg.length = CAN_ENCODER_FEEDBACK_LENGTH;
  msg.data[0] = CAN_ENCODER_FEEDBACK;
  msg.data[1] = one_to_n_bytes(encoder_value,0);
  msg.data[2] = one_to_n_bytes(encoder_value,1);
  msg.data[3] = one_to_n_bytes(encoder_value,2);
  msg.data[4] = one_to_n_bytes(encoder_value,3);
  can_tx_enqueue(msg);
}

bool encoder::is_encoder_working()
{
	return is_encoder_work;
}

encoder::~encoder(void)
{

}
