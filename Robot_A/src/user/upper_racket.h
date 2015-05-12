#ifndef	__UPPER_RACKET
#define	__UPPER_RACKET

#include <stdbool.h>
#include "stm32f10x.h"
#include "gpio.h"
#include "ticks.h"

#define	UPPER_RACKET_VALVE_COUNT			2
#define	UPPER_RACKET_VALVES 					{&PD8,&PD9}


#define	UPPER_RACKET_HIT_DELAY				500			/*!< The time that the hit last (valve-on time), in ms */
#define	UPPER_RACKET_POST_DELAY				1000		/*!< The time that CANNOT re-hit after a hit*/


void upper_racket_init(void);
void upper_racket_e_stop(void);
bool upper_racket_hit(u16 pre_delay);
void upper_racket_update(void);




#endif	/* __UPPER_RACKET */
