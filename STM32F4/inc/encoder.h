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

GPIO* const PHASE_A = &PB6;
GPIO* const PHASE_B = &PB7;


class encoder
{
public:
	encoder(GPIO* const phaseA, GPIO* const phaseB, TIM_TypeDef* const timer);
	int get_encoder_value(void);
	int get_change_of_encoder(void);
	virtual void refresh(void);
	void send_feedback(void);
	virtual ~encoder(void);

	bool is_encoder_work;
protected:
	const int this_id;

private:
	TIM_TypeDef* const m_TIM;
	int flowing_flag;
	int vel;
	int encoder_value;
	static const int SOURCE_ENCODER_INIT_VAL = 32768;

};



#endif /* ENCODER_H_ */
