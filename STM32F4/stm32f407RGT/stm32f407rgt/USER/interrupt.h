#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4xx_conf.h"
#include "buzzer.h"


/*
the number in gpio pin cant be the same if you want multi gpio interrupt
*/
void init_gpio_interrupt(const GPIO* gpio, EXTITrigger_TypeDef trigger_type);
