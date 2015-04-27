#include "auto_timer.h"
#include "ticks.h"
#include "wheel_base.h"
#include "wheel_base_pid.h"
#include "racket_control.h"
#include "nec_mb.h"

static int timer_ticks = 0;
const static int TIMER_BEGIN_TIME = 0;
const static int PID_OFF_TIME = 1000;
const static int SERVING_START_TIME = 5000;
const static int SERVING_END_TIME = 5800;
const static int STOP_TIMER_TIMEOUT = 200;

static bool calibrate = false;
static bool serve = false;
static bool timer_enabled = false;
static u8 mode = -1;

static void auto_timer_init(u8 timer_mode)
{
	timer_ticks = get_full_ticks();
	wheel_base_pid_on();
	set_starting_pos();
	calibrate = false;
	timer_enabled = true;
	disable_ultrasonic_sensor();
	mode = timer_mode;
}

static void auto_timer_update()
{
	for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) {
		if (nec_get_msg(i)->state != 0 && nec_get_msg(i)->address == 0x04 && nec_get_msg(i)->command == 0x19) {
				stop_all_motors();
		}
	}
	// do not update timer if not enabled
	if (!timer_enabled) { return; }
	if (wheel_base_get_last_manual_timer() != 0 && get_full_ticks() - wheel_base_get_last_manual_timer() > STOP_TIMER_TIMEOUT) {
		timer_enabled = false;
	}
	// config
	if (mode == 0) {
		buzzer_control_note(5, 50, NOTE_C, 7);
		if (get_full_ticks() - timer_ticks < TIMER_BEGIN_TIME) {
			// do nothing before timer begins
		} else if (get_full_ticks() - timer_ticks < PID_OFF_TIME) {
				set_serving_pos();
		} else if (get_full_ticks() - timer_ticks < SERVING_START_TIME) {
			// time before serving start, used to calibrate
			if (!calibrate) {
				racket_calibrate();
				calibrate = true;
			}
		} else if (get_full_ticks() - timer_ticks < SERVING_END_TIME) {
			// serve
			if (!serve) {
				serving();
				serve = true;
			}
		} else {
			// after serve
			set_after_serve_pos();
			enable_ultrasonic_sensor();
			timer_enabled = false;
		}
	} else if (mode == 1) {
		for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) {
			if (nec_get_msg(i)->state != 0 && nec_get_msg(i)->address == 0x04) {
				switch (nec_get_msg(i)->command) {
					case 0x12: set_serving_pos(); break;
					case 0x13: racket_calibrate(); break;
					case 0x14: serving(); break;
					case 0x15: set_after_serve_pos(); enable_ultrasonic_sensor(); break;
					case 0x16: upper_hit(); break;
					case 0x17: timer_enabled = false; break;
					default: break;
				}
			}
		}
	}
}

auto_timer robot_timer = {
	auto_timer_init,
	auto_timer_update
};
