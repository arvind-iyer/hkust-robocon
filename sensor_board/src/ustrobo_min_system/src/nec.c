#include "nec.h"

static NEC_TypeDef nec_devices[] = {
	{.gpio = &PA4},
	{.gpio = &PA5},
	{.gpio = &PA6},
	{.gpio = &PA7},
};

//typedef struct {
//  u16 address;
//  void (*fx)(u16 command);
//} NEC_HANDLER_PAIR;

/*
static u16 cont_on_count = 0;
static u16 cont_off_count = 0;
static u16 cont_on_falling = 0;
static u16 cont_off_falling = 0;

static u16 cont_on_max = 0;
static u16 cont_off_max = 0;

static u8 nec_data_reading_state = 0; // waiting for data state 
static u8 nec_data_current_bit = 0;
static u8 nec_data_current_buffer = 0;

static NEC_Data_TypeDef nec_last_data = (NEC_Data_TypeDef)-1;
static NEC_Data_TypeDef nec_raw_data[4] = {0};
*/

#define NEC_REPEAT_CYCLE        4
/*
static u8 nec_current_repeating_id = 0;

static NEC_Msg last_msg = {0, 0};
static NEC_Msg current_msg = {0, 0}; 
*/

typedef enum {
  NEC_RAW_ADDRESS_LOW,
  NEC_RAW_ADDRESS_HIGH, /** Complement of low **/
  NEC_RAW_COMMAND_LOW,
  NEC_RAW_COMMAND_HIGH  /** Complement of low **/
} nec_raw_data_id;

static NEC_STATE nec_state = NEC_NULL;




typedef struct {
  u16 min, max;
} NEC_RANGE;

static const u16 NEC_PULSE_LOW_MAX = 3400;
static const NEC_RANGE
  NEC_BURST_ON_RANGE = {280, 320},
  NEC_BURST_OFF_RANGE = {130, 185},
  
  NEC_DATA_ON_RANGE = {10, 35},
  NEC_DATA_ONE_OFF_RANGE = {35, 75},
  NEC_DATA_ZERO_OFF_RANGE = {5, 25},
  
  NEC_REPEAT_START_ON_RANGE = {10, 35},
  NEC_REPEAT_START_OFF_RANGE = {1200, 1600};

static const NEC_RANGE NEC_REPEAT_RANGE[NEC_REPEAT_CYCLE] = {
  /* + */ {285, 315}, /* - */ {60, 80},
  /* + */ {10, 35},   /* - */ {3200, 3400}
};  


static u8 nec_in_range(NEC_RANGE range, u16 val)
{
    return val >= range.min && val <= range.max;
}


void nec_init(void)
{
	for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) {
		gpio_init(nec_devices[i].gpio, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING, 1);
		nec_devices[i].state = NEC_NULL;
		nec_devices[i].current_msg = NEC_NullMsg;
		nec_devices[i].last_msg = NEC_NullMsg;
		nec_devices[i].data_reading_state = 0;
	}
  //gpio_init(NEC_GPIO, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING, 1);
  
  
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      									// TimeBase is for timer setting   > refer to P. 344 of library

	RCC_APB1PeriphClockCmd(NEC_RCC , ENABLE);

	
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
	
  //last_msg = NEC_NullMsg;
  //current_msg = NEC_NullMsg;
}

static void nec_set_msg(NEC_Msg* msg, u16 address, u16 command)
{
    msg->address = address;
    msg->command = command;
}

void nec_state_reset(u8 id)
{
	nec_devices[id].state = NEC_NULL;
	nec_devices[id].current_msg = NEC_NullMsg;
  /*nec_state = NEC_NULL;
  current_msg = NEC_NullMsg;*/
}

static u8 nec_process_data(u8 i)
{
	NEC_TypeDef* nec_device = &nec_devices[i];
  if (nec_device->data_reading_state) {
    if (nec_in_range(NEC_DATA_ON_RANGE, nec_device->cont_on_falling)) {
      nec_device->data_reading_state = 0;
      return 1;
    } else {
      nec_state_reset(i);
      return 0;
    }
  } else {
    u16 data = (u16)-1;  // Invalid data
    if (nec_in_range(NEC_DATA_ONE_OFF_RANGE, nec_device->cont_off_falling)) {
      data = 1;
    } else if (nec_in_range(NEC_DATA_ZERO_OFF_RANGE, nec_device->cont_off_falling)) {
      data = 0;
    }
    
    if (data == 0 || data == 1) {
      // Data starts from LSB to MSB
      for (u8 i = 0; i < nec_device->data_current_bit; ++i) {
        data *= 2;
      }
      nec_device->data_current_buffer += data;
      nec_device->data_current_bit++;
      nec_device->data_reading_state = 1;
      return 1;
      
    } else {
      nec_state_reset(i);
      return 0;
    }
  }  
}



static void nec_buffer_reset(u8 i, u8 next_reading_state)
{
	NEC_TypeDef* nec_device = &nec_devices[i];
  nec_device->data_current_bit = 0;
  nec_device->data_current_buffer = 0;
  nec_device->data_reading_state = next_reading_state;  
}

static void nec_loop(u8 i) 
{
		NEC_TypeDef* nec_device = &nec_devices[i];
    u8 input = gpio_read_input(NEC_GPIO);
    if (!input) {
      ++nec_device->cont_on_count;
			/*
      if (nec_device->cont_on_count > cont_on_max) {
        cont_on_max = nec_device->cont_on_count;
      }
			*/
      if (nec_device->cont_off_count) {
        nec_device->cont_off_falling = nec_device->cont_off_count;
        printf("-%d", nec_device->cont_off_falling);
        nec_device->cont_off_count = 0;
      } else {
        nec_device->cont_off_falling = 0;
      }
    } else {
      if (nec_device->cont_off_count < NEC_PULSE_LOW_MAX) {
        ++nec_device->cont_off_count;
      } else {
        // Reset
        nec_device->cont_off_count = 0;
        nec_state_reset(i);
      }
      
			/*
      if (nec_device->cont_off_count > nec_device->cont_off_max) {
        nec_device->cont_off_max = nec_device->cont_off_count; 
      }
			*/
			
      if (nec_device->cont_on_count) {
        nec_device->cont_on_falling = nec_device->cont_on_count;
        printf("+%d", nec_device->cont_on_falling);
        nec_device->cont_on_count = 0;
      } else {
        nec_device->cont_on_falling = 0; 
      }
    }
    
    if (nec_device->cont_on_falling ^ nec_device->cont_off_falling) {
      switch (nec_state) {
        case NEC_NULL:
          if (nec_in_range(NEC_BURST_ON_RANGE, nec_device->cont_on_falling)) { 
            // Burst on
            nec_state = NEC_BURST;
          } else {
            // Wrong signal
            nec_state_reset(i);
          }
          break;
          
        case NEC_BURST:
          
          if (nec_in_range(NEC_BURST_OFF_RANGE, nec_device->cont_off_falling)) {
            // Burst confirmed, start reading address
            nec_state = NEC_ADDRESS_LOW;
            nec_device->data_current_bit = 0;
            nec_device->data_current_buffer = 0;
            nec_device->data_reading_state = 1; 
          } else {
            // Wrong signal
            nec_state_reset(i);
          }
        break;
          
        case NEC_ADDRESS_LOW:
          
          nec_process_data(i);
          if (nec_device->data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_ADDRESS_HIGH;
            nec_device->raw_data[NEC_RAW_ADDRESS_LOW] = nec_device->last_data = nec_device->data_current_buffer;
            
            // Buffers reset
            nec_buffer_reset(i, 1);
          }
        break;
          
        case NEC_ADDRESS_HIGH:
          //break;
          nec_process_data(i);
          if (nec_device->data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_COMMAND_LOW;
            nec_device->raw_data[NEC_RAW_ADDRESS_HIGH] = nec_device->last_data = nec_device->data_current_buffer;
            // Buffers reset
            nec_buffer_reset(i, 1); 
            
          }
        break;
          
        case NEC_COMMAND_LOW:
          
          nec_process_data(i);
          if (nec_device->data_current_bit >= NEC_DATA_BIT) {
            nec_device->state = NEC_COMMAND_HIGH;
            nec_device->raw_data[NEC_RAW_COMMAND_LOW] = nec_device->last_data = nec_device->data_current_buffer;
            // Buffers reset
            nec_buffer_reset(i, 1);
          }
        break;
          
        case NEC_COMMAND_HIGH:
          
          nec_process_data(i);
          if (nec_device->data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_REPEAT_START;
            nec_device->raw_data[NEC_RAW_COMMAND_HIGH] = nec_device->last_data = nec_device->data_current_buffer;
            // Buffers reset
            nec_buffer_reset(i, 1);
            
            // DATA VALID CHECK
            if ((nec_device->raw_data[NEC_RAW_ADDRESS_LOW] & NEC_DATA_MAX) == (~nec_device->raw_data[NEC_RAW_ADDRESS_HIGH] & NEC_DATA_MAX)  \
              && (nec_device->raw_data[NEC_RAW_COMMAND_LOW] & NEC_DATA_MAX) == (~nec_device->raw_data[NEC_RAW_COMMAND_HIGH] & NEC_DATA_MAX)) {
              // Successful data
              nec_set_msg(&nec_device->current_msg, nec_device->raw_data[NEC_RAW_ADDRESS_LOW], nec_device->raw_data[NEC_RAW_COMMAND_LOW]);
              nec_set_msg(&nec_device->last_msg, nec_device->raw_data[NEC_RAW_ADDRESS_LOW], nec_device->raw_data[NEC_RAW_COMMAND_LOW]); 
              buzzer_control_note(1, 100, (MUSIC_NOTE_LETTER) (nec_device->raw_data[NEC_RAW_COMMAND_LOW] % 12 + 1), 6);
            } else {
              // Invalid data
              nec_state_reset(i); 
            }
            
            
          }
        break;
          
        case NEC_REPEAT_START:
          if (nec_in_range(NEC_REPEAT_START_ON_RANGE, nec_device->cont_on_falling)) {
            /*** Repeat start on detected ***/
          } else if (nec_in_range(NEC_REPEAT_START_OFF_RANGE, nec_device->cont_off_falling)) {
            /*** Repeat start off detected ***/
            nec_device->current_repeating_id = 0;
            nec_device->state = NEC_REPEATING;
          } else {
            nec_state_reset(i);  
          }
        break;
          
        case NEC_REPEATING:
          if (nec_device->current_repeating_id % 2 == 0) {
            // On pulse
            if (nec_in_range(NEC_REPEAT_RANGE[nec_device->current_repeating_id], nec_device->cont_on_falling)) {
              nec_device->current_repeating_id = (nec_device->current_repeating_id + 1) % NEC_REPEAT_CYCLE;
            } else {
              // Repeat ends or wrong data
              nec_state_reset(i);
            }
          } else {
            // Off pulse
            if (nec_in_range(NEC_REPEAT_RANGE[nec_device->current_repeating_id], nec_device->cont_off_falling)) {
              nec_device->current_repeating_id = (nec_device->current_repeating_id + 1) % NEC_REPEAT_CYCLE;
            } else {
              // Repeat ends or wrong data
              nec_state_reset(i);
            }
          }
        
        break;
      }
    }
}


NEC_IRQHandler
{
  
  if (TIM_GetITStatus(NEC_TIM, TIM_IT_Update) != RESET) {
    TIM_ClearFlag(NEC_TIM, TIM_IT_Update);
		for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) { 
			nec_loop(i);
			
		}
  }
 
}

/*
u16 get_nec_cont_on_max(u8 i)
{
  return nec_devices[i].cont_on_max;
}

u16 get_nec_cont_off_max(u8 i)
{
  return nec_devices[i].cont_off_max;
}
*/

NEC_STATE get_nec_state(u8 i)
{
  return nec_devices[i].state;
}

NEC_Data_TypeDef* get_nec_raw_data(u8 i)
{
  return nec_devices[i].raw_data;
}

NEC_Data_TypeDef get_nec_last_data(u8 i)
{
  return nec_devices[i].last_data;
}

NEC_Msg get_nec_last_msg(u8 i)
{
  return nec_devices[i].last_msg;
}

NEC_Msg get_nec_current_msg(u8 i)
{
   return nec_devices[i].current_msg;
}

void nec_can_tx(u8 i)
{
	CAN_MESSAGE msg;
	
	msg.length = 3;
	msg.id = NEC_CAN_ID + i;
	msg.data[0] = nec_devices[i].current_msg.address;
	msg.data[1] = nec_devices[i].current_msg.command;
	can_tx_enqueue(msg); 
}


