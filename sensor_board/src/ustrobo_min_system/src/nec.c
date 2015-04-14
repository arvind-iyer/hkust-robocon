#include "nec.h"

typedef struct {
  u16 address;
  void (*fx)(u16 command);
} NEC_HANDLER_PAIR;


static u16 cont_on_count = 0;
static u16 cont_off_count = 0;
static u16 cont_on_falling = 0;
static u16 cont_off_falling = 0;

static u16 cont_on_max = 0;
static u16 cont_off_max = 0;

static u8 nec_data_reading_state = 0; /** waiting for data state **/
static u8 nec_data_current_bit = 0;
static u8 nec_data_current_buffer = 0;

static NEC_Data_TypeDef nec_last_data = (NEC_Data_TypeDef)-1;
static NEC_Data_TypeDef nec_raw_data[4] = {0};
typedef enum {
  NEC_RAW_ADDRESS_LOW,
  NEC_RAW_ADDRESS_HIGH, /** Complement of low **/
  NEC_RAW_COMMAND_LOW,
  NEC_RAW_COMMAND_HIGH  /** Complement of low **/
} nec_raw_data_id;

static NEC_STATE nec_state = NEC_NULL;


#define NEC_REPEAT_CYCLE        4
static u8 nec_current_repeating_id = 0;

static NEC_Msg last_msg = {0, 0};
static NEC_Msg current_msg = {0, 0}; 

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
  gpio_init(NEC_GPIO, GPIO_Speed_2MHz, GPIO_Mode_IN_FLOATING, 1);
  
  
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
	
  last_msg = NEC_NullMsg;
  current_msg = NEC_NullMsg;
}

static void nec_set_msg(NEC_Msg* msg, u16 address, u16 command)
{
    msg->address = address;
    msg->command = command;
}

void nec_state_reset(void)
{
  nec_state = NEC_NULL;
  current_msg = NEC_NullMsg;
}

static u8 nec_process_data(void)
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
    } else if (nec_in_range(NEC_DATA_ZERO_OFF_RANGE, cont_off_falling)) {
      data = 0;
    }
    
    if (data == 0 || data == 1) {
      // Data starts from LSB to MSB
      for (u8 i = 0; i < nec_data_current_bit; ++i) {
        data *= 2;
      }
      nec_data_current_buffer += data;
      nec_data_current_bit++;
      nec_data_reading_state = 1;
      return 1;
      
    } else {
      nec_state_reset();
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
        printf("-%d", cont_off_falling);
        cont_off_count = 0;
      } else {
        cont_off_falling = 0;
      }
    } else {
      if (cont_off_count < NEC_PULSE_LOW_MAX) {
        ++cont_off_count;
      } else {
        // Reset
        cont_off_count = 0;
        nec_state_reset();
      }
      
      if (cont_off_count > cont_off_max) {
        cont_off_max = cont_off_count;
      }
      if (cont_on_count) {
        cont_on_falling = cont_on_count;
        printf("+%d", cont_on_falling);
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
          } else {
            // Wrong signal
            nec_state_reset();
          }
          break;
          
        case NEC_BURST:
          
          if (nec_in_range(NEC_BURST_OFF_RANGE, cont_off_falling)) {
            // Burst confirmed, start reading address
            nec_state = NEC_ADDRESS_LOW;
            nec_data_current_bit = 0;
            nec_data_current_buffer = 0;
            nec_data_reading_state = 1;
          } else {
            // Wrong signal
            nec_state_reset();
          }
        break;
          
        case NEC_ADDRESS_LOW:
          
          nec_process_data();
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_ADDRESS_HIGH;
            nec_raw_data[NEC_RAW_ADDRESS_LOW] = nec_last_data = nec_data_current_buffer;
            
            // Buffers reset
            nec_buffer_reset(1);
          }
        break;
          
        case NEC_ADDRESS_HIGH:
          //break;
          nec_process_data();
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_COMMAND_LOW;
            nec_raw_data[NEC_RAW_ADDRESS_HIGH] = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
            
          }
        break;
          
        case NEC_COMMAND_LOW:
          
          nec_process_data();
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_COMMAND_HIGH;
            nec_raw_data[NEC_RAW_COMMAND_LOW] = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
          }
        break;
          
        case NEC_COMMAND_HIGH:
          
          nec_process_data();
          if (nec_data_current_bit >= NEC_DATA_BIT) {
            nec_state = NEC_REPEAT_START;
            nec_raw_data[NEC_RAW_COMMAND_HIGH] = nec_last_data = nec_data_current_buffer;
            // Buffers reset
            nec_buffer_reset(1);
            
            // DATA VALID CHECK
            if ((nec_raw_data[NEC_RAW_ADDRESS_LOW] & NEC_DATA_MAX) == (~nec_raw_data[NEC_RAW_ADDRESS_HIGH] & NEC_DATA_MAX)  \
              && (nec_raw_data[NEC_RAW_COMMAND_LOW] & NEC_DATA_MAX) == (~nec_raw_data[NEC_RAW_COMMAND_HIGH] & NEC_DATA_MAX)) {
              // Successful data
              nec_set_msg(&current_msg, nec_raw_data[NEC_RAW_ADDRESS_LOW], nec_raw_data[NEC_RAW_COMMAND_LOW]);
              nec_set_msg(&last_msg, nec_raw_data[NEC_RAW_ADDRESS_LOW], nec_raw_data[NEC_RAW_COMMAND_LOW]); 
              buzzer_control_note(1, 100, (MUSIC_NOTE_LETTER) (nec_raw_data[NEC_RAW_COMMAND_LOW] % 12 + 1), 6);
            } else {
              // Invalid data
              nec_state_reset(); 
            }
            
            
          }
        break;
          
        case NEC_REPEAT_START:
          if (nec_in_range(NEC_REPEAT_START_ON_RANGE, cont_on_falling)) {
            /*** Repeat start on detected ***/
          } else if (nec_in_range(NEC_REPEAT_START_OFF_RANGE, cont_off_falling)) {
            /*** Repeat start off detected ***/
            nec_current_repeating_id = 0;
            nec_state = NEC_REPEATING;
          } else {
            nec_state_reset(); 
          }
        break;
          
        case NEC_REPEATING:
          if (nec_current_repeating_id % 2 == 0) {
            // On pulse
            if (nec_in_range(NEC_REPEAT_RANGE[nec_current_repeating_id], cont_on_falling)) {
              nec_current_repeating_id = (nec_current_repeating_id + 1) % NEC_REPEAT_CYCLE;
            } else {
              // Repeat ends or wrong data
              nec_state_reset();
            }
          } else {
            // Off pulse
            if (nec_in_range(NEC_REPEAT_RANGE[nec_current_repeating_id], cont_off_falling)) {
              nec_current_repeating_id = (nec_current_repeating_id + 1) % NEC_REPEAT_CYCLE;
            } else {
              // Repeat ends or wrong data
              nec_state_reset();
            }
          }
        
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

NEC_STATE get_nec_state(void)
{
  return nec_state;
}

NEC_Data_TypeDef* get_nec_raw_data(void)
{
  return nec_raw_data;
}

NEC_Data_TypeDef get_nec_last_data(void)
{
  return nec_last_data;
}

NEC_Msg get_nec_last_msg(void)
{
  return last_msg;
}

NEC_Msg get_nec_current_msg(void)
{
    return current_msg;
}

void nec_can_tx(void)
{
	CAN_MESSAGE msg;
	
	msg.length = 3;
	msg.id = NEC_CAN_ID;
	msg.data[0] = (u8) get_nec_state();
	msg.data[1] = current_msg.address;
	msg.data[2] = current_msg.command;
	can_tx_enqueue(msg); 
}


