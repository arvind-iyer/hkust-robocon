#ifndef	__US_AUTO_H
#define	__US_AUTO_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "ultrasonic_mb.h"
#include "ticks.h"

#define	US_DETECT_RANGE_MIN								5	
#define	US_DETECT_RANGE_MAX								1500

#define	US_DETECT_PROTECTION_TIME_MS			500
#define	US_DETECT_E_STOP_CONT_MS					300		/** Detection after this time is ignored (cumulatively) **/

#define	US_AUTO_DEVICE_COUNT							15


#define	US_AUTO_FRONT_BAR									{0,1,2,3,4,5,6,7}
#define	US_AUTO_UPPER_BAR									{8,9,10,11}
#define	US_AUTO_FRONT_BAR_NOISE						{12,13,14}

typedef enum {
	US_AUTO_NULL,
	US_AUTO_HIT,
	US_AUTO_HITTING,
	US_AUTO_E_STOP_HIT
}	US_AUTO_RESPONSE;

void us_auto_update(void);
u16 us_get_detection_val(void);
US_AUTO_RESPONSE us_auto_get_response(void); 


#endif	/* __US_AUTO_H */
