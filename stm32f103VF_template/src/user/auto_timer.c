#include "auto_timer.h"
#include "ticks.h"
#include "stm32f10x.h"
#include "wheel_base.h"
#include "wheel_base_pid.h"
#include "racket_control.h"

static int timer_ticks = 0;
const static int TIMER_BEGIN_TIME = 7000;
const static int PID_OFF_TIME = 10000;
const static int SERVING_START_TIME = 15500;
const static int SERVING_END_TIME = 16500;
const static int STOP_TIMER_TIMEOUT = 200;

static bool calibrate = false;
static bool serve = false;
static bool timer_enabled = false;

static void auto_timer_init()
{
	timer_ticks = get_full_ticks();
	wheel_base_pid_on();
	set_starting_pos();
	calibrate = false;
	timer_enabled = true;
	disable_ultrasonic_sensor();
}

static void auto_timer_update()
{
	// do not update timer if not enabled
	if (!timer_enabled) { return; }
	if (wheel_base_get_last_manual_timer() != 0 && get_full_ticks() - wheel_base_get_last_manual_timer() > STOP_TIMER_TIMEOUT) {
		timer_enabled = false;
	}
	// config
	buzzer_control_note(5, 50, NOTE_C, 7);
	
	if (get_full_ticks() - timer_ticks < TIMER_BEGIN_TIME) {
	} else if (get_full_ticks() - timer_ticks < PID_OFF_TIME) {
		if (!calibrate) {
			racket_calibrate();
			set_serving_pos();
			calibrate = true;
		}
	} else if (get_full_ticks() - timer_ticks < SERVING_START_TIME) {
	} else if (get_full_ticks() - timer_ticks < SERVING_END_TIME) {
		if (!serve) {
			serving();
			serve = true;
		}
	} else {
		set_after_serve_pos();
		enable_ultrasonic_sensor();
		timer_enabled = false;
	}
}

auto_timer robot_timer = {
	auto_timer_init,
	auto_timer_update
};
