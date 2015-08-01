/*
 * encoder.h
 *
 *  Created on: 5 Apr, 2015
 *      Author: William
 */
#include "gpio.h"
#include "timer.h"
#include "can_motor.h"

#ifndef ENCODER_H_
#define ENCODER_H_

GPIO* const encoder_A = &PC6;
GPIO* const encoder_B = &PC7;
// Warning: encoder interface MUST use channel 1 (and 2 with another phase).
TIMER* const encoder_TIM = &TIM3Ch1;


class encoder
{
public:
	encoder(GPIO* const phaseA, GPIO* const phaseB, TIMER* const timer);
	int get_encoder_value(void);
	int get_change_of_encoder(void);
	virtual void refresh(void);
	void send_feedback(void);
	bool get_encoder_working_state();
	virtual ~encoder(void);

protected:
	const int this_id;
	bool is_encoder_work;

private:
	TIMER* const m_TIM;
	int flowing_flag;
	int vel;
	int encoder_value;
	static const int SOURCE_ENCODER_INIT_VAL = 32768;

};



#endif /* ENCODER_H_ */
