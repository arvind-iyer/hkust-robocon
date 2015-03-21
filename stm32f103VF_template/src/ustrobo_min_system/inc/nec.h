#ifndef __NEC_H
#define __NEC_H

#include "stm32f10x.h" 
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "gpio.h"
#include "buzzer.h"
#include "led.h"
#include "usart.h"

/*
#define NEC_TIM							TIM9
#define NEC_RCC							RCC_APB2Periph_TIM9
#define NEC_IRQn            TIM1_BRK_TIM9_IRQn
#define NEC_IRQHandler      void TIM1_BRK_TIM9_IRQHandler(void)
*/

#define NEC_TIM							  TIM10
#define NEC_RCC							  RCC_APB2Periph_TIM10
#define NEC_IRQn						  TIM1_UP_TIM10_IRQn
#define NEC_IRQHandler			  void TIM1_UP_TIM10_IRQHandler(void)
  
#define NEC_FREQUENCY       38000 /**38000**/
#define NEC_GPIO            ((GPIO*) &PC6)
#define NEC_DATA_BIT        8
typedef enum {
  NEC_NULL,
  NEC_BURST, /* BURST ON DETECTED */
  NEC_ADDRESS_LOW,
  NEC_ADDRESS_HIGH,
  NEC_COMMAND_LOW,
  NEC_COMMAND_HIGH,
  NEC_REPEAT
} NEC_STATE;

/**
  uart_init(COM1, 115200);
  
  u16 ticks_img = (u16)-1;
  system_start(1200);

  while(1) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      led_control(LED_D2, !gpio_read_input(&PC6));
      tft_clear();
      tft_prints(0,0,"on_max: %d", get_nec_cont_on_max());
      tft_prints(0,1,"off_max: %d", get_nec_cont_off_max());
      tft_prints(0,2,"state:%d", get_nec_state());
      tft_prints(0,3,"last_data:%X", get_nec_last_data());
      tft_prints(0,4,"{%X, %X} {%X, %X}", nec_last_address_low, nec_last_address_high, nec_last_command_low, nec_last_command_high);
      
      tft_update();
    }
  }
	**/

void nec_init(void);
u16 get_nec_cont_on_max(void);
u16 get_nec_cont_off_max(void);
u16 get_nec_state(void);
u16 get_nec_last_data(void);

extern u16 nec_last_address_low;
extern u16 nec_last_address_high;
extern u16 nec_last_command_low;
extern u16 nec_last_command_high;

#endif  /* __NEC_H */
