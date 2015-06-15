#include "us_auto.h"

static u32 last_detection_full_ticks = 0;
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
	

	static const u8 front_bar[] = US_AUTO_FRONT_BAR;
	static const u8 upper_bar[] = US_AUTO_UPPER_BAR;
	static const u8 front_bar_noise[] = US_AUTO_FRONT_BAR_NOISE;
	
	u8 consective_count = 0;
	for (u8 i = 0; i < sizeof(front_bar) / sizeof(u8); ++i) {
		u16 val = us_get_distance(front_bar[i]);
		if (val >= 100 && val <= 1500) {
			++consective_count;
		} else {
			consective_count = 0;
		}
		
		if (consective_count >= 2) {
			detection_flag = true;
			break;
		}
	}
	
	
	for (u8 i = 0; i < sizeof(upper_bar) / sizeof(u8); ++i) {
		u16 val = us_get_distance(upper_bar[i]);
		if (val >= 10 && val <= 1000) {
			detection_flag = true;
			break;
		}
	}
	
	u8 noise_count = 0;
	for (u8 i = 0; i < sizeof(front_bar_noise) / sizeof(u8); ++i) {
		u16 val = us_get_distance(front_bar_noise[i]);
		if (val >= 10 && val <= 1500) {
			// Noise
			++noise_count;
			detection_flag = true;
		}
	}
	
	if (detection_flag) {
		if (noise_count >= 1) {
			response = US_AUTO_E_STOP_HIT;
			last_e_stop_full_ticks = get_full_ticks();
		} else if (last_detection_full_ticks && last_detection_full_ticks + US_DETECT_PROTECTION_TIME_MS <= get_full_ticks()) {
			// On for too long
			//response = US_AUTO_E_STOP_HIT;
			//last_detection_full_ticks = get_full_ticks();
			//last_e_stop_full_ticks = get_full_ticks();
		} else if (!last_detection_full_ticks) {
			response = US_AUTO_HIT;
			last_detection_full_ticks = get_full_ticks();
		} else {
			response = US_AUTO_HITTING;
		}	
	} else {
		response = US_AUTO_NULL; 
		last_detection_full_ticks = 0;
	}
	
	
	if (last_e_stop_full_ticks + US_DETECT_E_STOP_CONT_MS > get_full_ticks()) {
		response = US_AUTO_E_STOP_HIT;
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

