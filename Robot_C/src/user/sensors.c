#include "sensors.h"


// system variables
bool short_distance_detected=0;	// 200 to 800 range distance detected by ultrasonic
u32 last_ultra_detect_time=0;

bool ultra_detect_shuttle(void)	//200~800 for robot D
{
	
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
			// only cares about connected ultrasonic sensors
			if (us_get_distance(i)!=0)
			{
				// checks for range of distance. Rising edge.
				if (us_get_distance(i)<1500 && us_get_distance(i)>400)
				{
					log("ult_detect",us_get_distance(i));
					short_distance_detected=1;
					return 0;
				}
			}
		}
		
		//
		//#error Hong Joon did not let you to compile
		// Only returns true on falling edge. Once detected, it ignores all other stuff for a second.
		if (short_distance_detected &&( last_ultra_detect_time+1500<get_full_ticks()))
		{
			short_distance_detected=0;
			last_ultra_detect_time=get_full_ticks();
			return 1;
		}
		short_distance_detected=0;
		// all otherwise, return 0
		return 0;
}

void sensors_update(void)
{
	
	if (ultra_detect_shuttle())
	{
		racket_delayed_hit(0);
	}
	
}
