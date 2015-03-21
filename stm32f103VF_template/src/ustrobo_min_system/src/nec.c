#include "nec.h"

static u16 cont_on_count = 0;
static u16 cont_off_count = 0;
static u16 cont_on_falling = 0;
static u16 cont_off_falling = 0;

static u16 cont_on_max = 0;
static u16 cont_off_max = 0;

static u16 nec_data_reading_state = 0; /** waiting for data state **/
static u16 nec_data_current_bit = 0;
static u16 nec_data_current_buffer = 0;

static u16 nec_last_data = (u16)-1;

u16 nec_last_address_low = 0;
u16 nec_last_address_high = 0;
u16 nec_last_command_low = 0;
u16 nec_last_command_high = 0;

static NEC_STATE nec_state = NEC_NULL;


typedef struct {
  u16 min, max;
} NEC_RANGE;

static const NEC_RANGE
  NEC_BURST_ON_RANGE = {280, 320},
  NEC_BURST_OFF_RANGE = {130, 185},
  
  NEC_DATA_ON_RANGE = {10, 35},
  NEC_DATA_ONE_OFF_RANGE = {35, 75},
  NEC_DATA_ZERO_OFF_RANGE = {3, 25}
  ; 


static u8 nec_in_range(NEC_RANGE range, u16 val)
{
    return val >= range.min && val <= range.max;
}


void nec_init(void)
{
  gpio_init(NEC_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IPD, 1);
  
  
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library

	RCC_APB2PeriphClockCmd(NEC_RCC , ENABLE);
	
//	TICKS_TIM->PSC = SystemCoreClock / 1000000 - 1;		// Prescaler
//	TICKS_TIM->ARR = 1000;
//	TICKS_TIM->EGR = 1;
//	TICKS_TIM->SR = 0;
//	TICKS_TIM->DIER = 1;
//	TICKS_TIM->CR1 = 1;
	
	TIM_TimeBaseStructure.TIM_Period = 10;	                 				       // Timer period, 1000 ticks in one second
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / (NEC_FREQUENCY*10) - 1;     // 72M/1M - 1 = 71
	TIM_TimeBaseInit(NEC_TIM, &TIM_TimeBaseStructure);      							 // this part feeds the parameter we set above
	
	TIM_ClearITPendingBit(NEC_TIM, TIM_IT_Update);												 // Clear Interrupt bits
	TIM_ITConfig(NEC_TIM, TIM_IT_Update, ENABLE);													 // Enable TIM Interrupt
	TIM_Cmd(NEC_TIM, ENABLE);																							 // Counter Enable

	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = NEC_IRQn; 
	NVIC_Init(&NVIC_InitStructure);
	
	//SysTick_Config(SystemCoreClock/1000);

}

void nec_state_reset(void)
{
  nec_state = NEC_NULL;
}

static u8 nec_process_data(u8 inverse_flag)
{
  if (nec_data_reading_state) {
    if (nec_in_range(NEC_DATA_ON_RANGE, cont_on_falling)) {
      nec_data_reading_state = 0;
      return 1;
    } else {
      nec_state_reset();
      return 0;
    }
  } else {
    u16 data = (u16)-1;  // Invalid data
    if (nec_in_range(NEC_DATA_ONE_OFF_RANGE, cont_off_falling)) {
      data = 1;
      if (inverse_flag) {data = 0;}
    } else if (nec_in_range(NEC_DATA_ZERO_OFF_RANGE, cont_off_falling)) {
      data = 0;
      if (inverse_flag) {data = 1;}
    }
    
    if (data == 0 || data == 1) {
      //for (u8 i = 0; + i < nec
      for (u8 i = 0; i < nec_data_current_bit; ++i) {
        data *= 2;
      }
      nec_data_current_buffer += data;
      nec_data_current_bit++;
      return 1;
      
    } else {
      //nec_state_reset();
      return 0;
    }
  }  
}



static void nec_buffer_reset(u8 next_reading_state)
{
  nec_data_current_bit = 0;
  nec_data_current_buffer = 0;
  nec_data_reading_state = next_reading_state;  
}


NEC_IRQHandler
{
  
  if (TIM_GetITStatus(NEC_TIM, TIM_IT_Update) != RESET) {
    TIM_ClearFlag(NEC_TIM, TIM_IT_Update);
    u8 input = gpio_read_input(NEC_GPIO);
    if (!input) {
      ++cont_on_count;
      if (cont_on_count > cont_on_max) {
        cont_on_max = cont_on_count;
      }
      if (cont_off_count) {
        cont_off_falling = cont_off_count;
        //uart_tx(COM1, "%d\n", cont_off_falling);
        cont_off_count = 0;
      } else {
        cont_off_falling = 0;
      }
    } else {
      ++cont_off_count;
      if (cont_off_count > cont_off_max) {
        cont_off_max = cont_off_count;
      }
      if (cont_on_count) {
        cont_on_falling = cont_on_count;
        //uart_tx(COM1, "'%d\n", cont_on_falling);
        cont_on_count = 0;
      } else {
        cont_on_falling = 0;
      }
    }
    
    if (cont_on_falling ^ cont_off_falling) {
      switch (nec_state) {
        case NEC_NULL:
          if (nec_in_range(NEC_BURST_ON_RANGE, cont_on_falling)) { 
            // Burst on
            nec_state = NEC_BURST;
          }
          break;
          
        case NEC_BURST:
          
          if (nec_in_range(NEC_BURST_OFF_RANGE, cont_off_falling)) {
            // Burst confirmed, start reading address
            nec_state = NEC_ADDRESS_LOW;
            nec_data_current_bit = 0;
            nec_data_current_buffer = 0;
            nec_data_reading_state = 1;
          }
        break;
          
        case NEC_ADDRESS_LOW:
          
          nec_process_data(0);
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_ADDRESS_HIGH;
            nec_last_address_low = nec_last_data = nec_data_current_buffer;
            
            // Buffers reset
            nec_buffer_reset(1);
          }
        break;
          
        case NEC_ADDRESS_HIGH:
          //break;
          nec_process_data(1);
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_COMMAND_LOW;
            nec_last_address_high = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
            
            // DATA VALID CHECK
            if (nec_last_address_high != nec_last_address_low) {
              nec_state_reset(); 
            }
          }
        break;
          
        case NEC_COMMAND_LOW:
          
          nec_process_data(0);
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_COMMAND_HIGH;
            nec_last_command_low = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
          }
        break;
          
        case NEC_COMMAND_HIGH:
          
          nec_process_data(1);
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_REPEAT;
            nec_last_command_high = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
            
            // DATA VALID CHECK
            if (nec_last_command_high != nec_last_command_low) {
              nec_state_reset(); 
            }
          }
        break;
          
        case NEC_REPEAT:
          
          nec_state_reset(); 
        break;
      }
    }
  }
 
}


u16 get_nec_cont_on_max(void)
{
  return cont_on_max;
}

u16 get_nec_cont_off_max(void)
{
  return cont_off_max;
}

u16 get_nec_state(void)
{
  return nec_state;
}

u16 get_nec_last_data(void)
{
  return nec_last_data;
}
