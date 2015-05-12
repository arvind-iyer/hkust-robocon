#include "us_auto.h"

u32 last_detection_full_ticks = 0;
static US_AUTO_RESPONSE response = US_AUTO_NULL;
static u16 us_detection = 0;
static u32 last_e_stop_full_ticks = 0;

static bool us_in_range(u8 id) 
{
	u16 dist = us_get_distance(id);
	return (dist >= US_DETECT_RANGE_MIN && dist <= US_DETECT_RANGE_MAX);
}


void us_auto_update(void)
{
	/*** Check in-range (2 or more consecutive) ***/
	bool detection_flag = false;
	u8 consecutive_count = 0;
	u16 min_val = 9999;
	
	for (u8 i = 0; i < US_AUTO_DEVICE_COUNT; ++i) {
		u16 val = us_get_distance(i);
		
		if (val >= US_DETECT_RANGE_MIN && val <= US_DETECT_RANGE_MAX) {
			// Detected
			++consecutive_count;
			if (val < min_val) {
				min_val = val;
			}
		} else {
			consecutive_count = 0;
			//min_val = 9999;
		}
		
		if (consecutive_count >= 2) {
			detection_flag = true;
			//break;
		}
	}
	
	// Checking... 
	if (detection_flag) {
		us_detection = min_val;
	
		if (!last_detection_full_ticks) {
			// Raising
			last_detection_full_ticks = get_full_ticks();
			if (last_e_stop_full_ticks + US_DETECT_E_STOP_CONT_MS <= get_full_ticks()) {
				response = US_AUTO_HIT;
			} else {
				// E-stop not cooled down
				response = US_AUTO_E_STOP_HIT;
				last_e_stop_full_ticks = get_full_ticks();
			}
			
		} else {
			// Check if the sensor is on for too long
			if (last_detection_full_ticks + US_DETECT_PROTECTION_TIME_MS <= get_full_ticks()) {
				// Stop
				response = US_AUTO_E_STOP_HIT;
				last_e_stop_full_ticks = get_full_ticks();
			} else {
				// On
				response = US_AUTO_HITTING;
			}
		}
	} else {
		// falling
		if (last_detection_full_ticks) {
			last_detection_full_ticks = 0;
			response = US_AUTO_NULL;
		}
	}
}

u16 us_get_detection_val(void)
{
	return us_detection;
}

US_AUTO_RESPONSE us_auto_get_response(void)
{
	return response;
}

