/*
 * Ticks.cpp
 *
 *  Created on: 2014�~8��3��
 *      Author: YunKei
 */

#include <Ticks.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <motor.h>

Ticks* _mTicks;
static int counter = 0;

void SysTick_Handler(void)
{
	if (++counter >= 5000) {
		counter = 0;
	}
	if (counter % 5 == 0) {
		_mTicks->TicksIncrement();
	}
	try {
		motor::get_instance(0)->pid_control(640, 12, 240);
	} catch (motor_error&) {
		return;
	}
}

uint16_t Ticks::getTimeout(){

	return timeoutCount;
}

Ticks::Ticks() : ticks(0), timeoutCount(0), seconds(0)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(SystemCoreClock / clock_frequency);
	_mTicks = this;
}

Ticks* Ticks::getInstance()
{
	return _mTicks;
}

bool Ticks::TicksComp(uint16_t period, uint16_t phaseShift, uint16_t ticksImg)
{
	if(ticksImg % period == phaseShift){
		return true;
	}
	else{
		return false;
	}
}

void Ticks::TicksIncrement()
{
	if (++ticks >= MAX_TICKS) {
		++seconds;
		ticks = 0;
	}
}

uint16_t Ticks::getTicks(){

	return ticks;
}

uint16_t Ticks::getFullTicks()
{
	return seconds * 1000 + ticks;
}

uint16_t Ticks::getSeconds()
{
	return seconds;
}

bool Ticks::Timeout(){
	return getTimeout() == 0 ? true : false;
}

void Ticks::setTimeout(uint16_t value){
	timeoutCount = value;
}
