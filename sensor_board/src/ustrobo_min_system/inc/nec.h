#ifndef __NEC_H
#define __NEC_H

#include "stm32f10x.h" 
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "gpio.h"
#include "buzzer.h"
#include "led.h"
#include "usart.h"
#include "can_protocol.h"

#define NEC_TIM							TIM3
#define NEC_RCC							RCC_APB1Periph_TIM3
#define NEC_IRQn            TIM3_IRQn
#define NEC_IRQHandler      void TIM3_IRQHandler(void)

#define	NEC_QUEUE_SIZE			300

#define NEC_FREQUENCY       38000
#define NEC_GPIO            ((GPIO*) &PA4)
#define NEC_DATA_BIT        8
#define NEC_DATA_MAX        0xFF   /** For 8 bit **/

#define	NEC_CAN_ID					0x220

#define	NEC_DEVICE_COUNT		8

#define	NEC_PULSE_MAX				1000

typedef u8 NEC_Data_TypeDef;

typedef enum {
  NEC_NULL = 0,
  NEC_BURST, /* BURST ON DETECTED */
  NEC_ADDRESS_LOW,
  NEC_ADDRESS_HIGH,
  NEC_COMMAND_LOW,
  NEC_COMMAND_HIGH,
  NEC_REPEAT_START,
  NEC_REPEATING,
} NEC_STATE;

typedef struct {
  NEC_Data_TypeDef address, command;
} NEC_Msg;

static const NEC_Msg NEC_NullMsg = {0,0};

typedef struct {
	const GPIO* gpio;
	NEC_STATE state;
	NEC_Msg last_msg, current_msg;
	u8 data_reading_state;
	u8 data_current_bit;
	u8 data_current_buffer;
	//u16 cont_on_count, cont_off_count;
	//u16 cont_on_falling, cont_off_falling;
	
	u16 pulse_width;
	u16 deque[NEC_QUEUE_SIZE];
	u16 deque_head, deque_tail;
	
	NEC_Data_TypeDef raw_data[4];
	NEC_Data_TypeDef last_data;
	u8 current_repeating_id;
}	NEC_TypeDef;
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
void nec_printf(void);
void nec_update(void);
u16 get_nec_cont_on_max(void);
u16 get_nec_cont_off_max(void);
NEC_STATE get_nec_state(u8 i);
NEC_Data_TypeDef* get_nec_raw_data(u8 i);  /** ARRAY **/
NEC_Data_TypeDef get_nec_last_data(u8 i); 


NEC_Msg get_nec_last_msg(u8 i);
NEC_Msg get_nec_current_msg(u8 i);
  
void nec_can_tx(u8 i);

#endif  /* __NEC_H */
