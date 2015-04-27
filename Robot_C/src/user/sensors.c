#include "sensors.h"



// system variables
u32 short_distance_detected_counter=0;	// 200 to 800 range distance detected by ultrasonic sensors array
u32 last_ultra_detect_time=0;

u32 ir_detected_counter = 0;			// one of the ir sensor array is detecting anything.
u32 last_ir_detect_time = 0;

bool ultra_detect_pulse_high()
{
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
			// only cares about connected ultrasonic sensors
			if (us_get_distance(i)!=0)
			{
				// checks for range of distance. Rising edge.
				if (us_get_distance(i)<ULTRA_DETECT_MAX_RANGE && us_get_distance(i)>ULTRA_DETECT_MIN_RANGE)
				{
					log("ult_pulse",short_distance_detected_counter);
					//log("ult_detect",us_get_distance(i));
					return 1;
				}
			}
		}
	return 0;
	
}

bool ir_detect_pulse_high()
{
	if (gpio_read_input(laser_sensor_1) || gpio_read_input(laser_sensor_2) || gpio_read_input(laser_sensor_3) || gpio_read_input(laser_sensor_4) || gpio_read_input(laser_sensor_5) || gpio_read_input(laser_sensor_6) || gpio_read_input(laser_sensor_7))
	{
		return 1;
	}
	return 0;
}

bool ultra_detect_shuttle(void)	//200~800 for robot D
{
	// for 1.5 seconds after sensor triggered hitting, it ignores any incoming data.
	if (last_ultra_detect_time+1500>get_full_ticks())
		return 0;
	
	if (ultra_detect_pulse_high())
	{
		short_distance_detected_counter+=1;
		return 0;
	}

	// if the program reaches this part, short_distance_detected is on falling edge.
	
	
	//#error Hong Joon did not let you to compile lolz
	// Only returns true on falling edge. Once detected, it ignores all other stuff for a second.
	// Accepts only the certain range of pulse.
	if (short_distance_detected_counter<SENSOR_PULSE_WIDTH_MAX && short_distance_detected_counter>SENSOR_PULSE_WIDTH_MIN)
	{
		//log ("ult_pulse",short_distance_detected_counter);
		short_distance_detected_counter=0;
		last_ultra_detect_time=get_full_ticks();
		return 1;
	}
	
		
		short_distance_detected_counter=0;
		// all otherwise, return 0
		return 0;
}

bool ir_detect_shuttle(void)
{
	
	
	
	// if one of the laser sensor is triggered, add to the counter
	if (ir_detect_pulse_high())
	{
		//log("ir_pulse",ir_detected_counter);
		ir_detected_counter+=1;
		//given the acceptable width of pulse, hit the bottom racket.
		// IT IS NOT A FALLING EDGE
		 if ( ir_detected_counter<SENSOR_PULSE_WIDTH_MAX && ir_detected_counter>SENSOR_PULSE_WIDTH_MIN)
		{
			log("ir_pulse",ir_detected_counter);
			last_ir_detect_time = get_full_ticks();
		}
		// for 1.5 seconds after sensor triggered hitting, it ignores any incoming data.
		if (last_ir_detect_time+1500>get_full_ticks())
			return 0;
		else	
			return 1;
	}
	
	// reset ir_detected_counter
	ir_detected_counter=0;
	
	return 0;
	
}

void sensors_update(void)
{
	
	if (ultra_detect_shuttle())
	{
		racket_delayed_hit(0);
		log("ultra_racket",1);
	}
	if (ROBOT=='C' && ir_detect_shuttle())
	{
		racket_down_hit();
		log("ir_racket", 1);
		last_ultra_detect_time = get_full_ticks()-1000;	// minor ultrasonic noise correction. when hitting by ir, disable ultrasonic for 500 ms
	}
}
