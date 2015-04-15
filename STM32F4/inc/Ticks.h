/*
 * Ticks.h
 *
 *  Created on: 2014�~8��3��
 *      Author: YunKei
 */

#ifndef TICKS_H_
#define TICKS_H_

#include <inttypes.h>
#include <stm32f4xx_it.h>

const uint16_t MAX_TICKS = 1000;
const int clock_frequency = 5000;

class Ticks{


	public:

		Ticks();
		static Ticks* getInstance();
		void TicksIncrement();
		bool TicksComp(uint16_t, uint16_t, uint16_t);
		uint16_t getTicks();
		uint16_t getSeconds();
		uint16_t getFullTicks();
		void setTimeout(uint16_t);
		bool Timeout();
		uint16_t getTimeout();

	private:
		uint16_t ticks;
		uint16_t seconds;
		uint16_t timeoutCount;
};

#endif /* TICKS_H_ */
