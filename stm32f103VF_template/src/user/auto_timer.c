#include "auto_timer.h"
#include "ticks.h"
#include "stm32f10x.h"
#include "wheel_base.h"
#include "wheel_base_pid.h"
#include "racket_control.h"

static int timer_ticks = 0;
const static int PID_OFF_TIME = 3000;
const static int PNEUMATIC_CLOSE_TIME = 4000;
const static int CONFIG_END_TIME = 5000;
const static int SERVING_START_TIME = 8500;
const static int SERVING_END_TIME = 9500;

bool calibrate = false;
bool serve = false;
bool timer_enabled = false;

void auto_timer_init()
{
	timer_ticks = get_full_ticks();
	wheel_base_pid_on();
	set_starting_pos();
	calibrate = false;
	timer_enabled = true;
}

void auto_timer_update()
{
	// do not update timer if not enabled
	if (!timer_enabled) { return; }
	// config
	if (get_full_ticks() - timer_ticks < PID_OFF_TIME) {
		if (!calibrate) {
			racket_calibrate();
			set_serving_pos();
			open_pneumatic();
			calibrate = true;
		}

	} else if (get_full_ticks() - timer_ticks < PNEUMATIC_CLOSE_TIME) {
		open_pneumatic();
	} else if (get_full_ticks() - timer_ticks < CONFIG_END_TIME) {
		close_pneumatic();
	} else if (get_full_ticks() - timer_ticks < SERVING_START_TIME) {
	} else if (get_full_ticks() - timer_ticks < SERVING_END_TIME) {
		if (!serve) {
			serving();
			serve = true;
		}
	} else {
		set_after_serve_pos();
		timer_enabled = false;
	}
	
}
