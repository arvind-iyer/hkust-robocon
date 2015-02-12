#include "game_counter.h"

static LED_TUBE LED_DIGIT[LED_DIGIT_COUNT][LED_SEGMENT_COUNT] = {
  {
    {&PB0, false},
    {&PA4, false},
    {&PA5, false},
    {&PA6, false},
    {&PA7, false},
    {&PC4, false},
    {&PC5, false}
  },
  
  {
    {&PC8, false},
    {&PB12, false},
    {&PB13, false},
    {&PB14, false},
    {&PB15, false},
    {&PC7, false},
    {&PC6, false}
  },
  
  {
    {&PB9, false}, 
    {&PC11, false},
    {&PC12, false},
    {&PB5, false},
    {&PB6, false},
    {&PB8, false},
    {&PB7, false}
  }
};

static const u8 DIGIT_MAP[][LED_SEGMENT_COUNT]={
    {1, 1, 1, 1, 1, 1, 0}, /*!< Digit 0 */
    {0, 1, 1, 0, 0, 0, 0}, /*!< Digit 1 */
    {1, 1, 0, 1, 1, 0, 1}, /*!< Digit 2 */
    {1, 1, 1, 1, 0, 0, 1}, /*!< Digit 3 */
    {0, 1, 1, 0, 0, 1, 1}, /*!< Digit 4 */
    {1, 0, 1, 1, 0, 1, 1}, /*!< Digit 5 */
    {1, 0, 1, 1, 1, 1, 1}, /*!< Digit 6 */
    {1, 1, 1, 0, 0, 0, 0}, /*!< Digit 7 */
    {1, 1, 1, 1, 1, 1, 1}, /*!< Digit 8 */
    {1, 1, 1, 1, 0, 1, 1}, /*!< Digit 9 */
    {0, 0, 0, 0, 0, 0, 0},  /*!< No digit */
    {0, 1, 1, 1, 1, 1, 0},  /*!< Letter U */
    {1, 1, 0, 0, 1, 1, 1},  /*!< Letter P */
};
  
static LED_TUBE LED_COLON = {&PB1, false};
void game_counter_init(void)
{
  // Initialize the LCD_DIGIT
  for (u8 digit_id = 0; digit_id < LED_DIGIT_COUNT; ++digit_id) {
    for (u8 segment = 0; segment < LED_SEGMENT_COUNT; ++segment) {
      LED_TUBE* tube = &LED_DIGIT[digit_id][segment];
      gpio_init(tube->gpio, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
      gpio_write(tube->gpio, (BitAction) 0);  // off
    }
  }
  
  // Colon
  gpio_init(LED_COLON.gpio, GPIO_Speed_2MHz, GPIO_Mode_Out_PP, 1);
  gpio_write(LED_COLON.gpio, (BitAction) 0);
}

void game_counter_all_off(void)
{
  // Iterate all the LCD_DIGIT
  for (u8 digit_id = 0; digit_id < LED_DIGIT_COUNT; ++digit_id) {
    for (u8 segment = 0; segment < LED_SEGMENT_COUNT; ++segment) {
      LED_TUBE* tube = &LED_DIGIT[digit_id][segment];
      gpio_write(tube->gpio, (BitAction)0);  // off
    }
  } 
  
  // Colon
  gpio_write(LED_COLON.gpio, (BitAction) 0);
}

void game_counter_tube_set(u8 digit_id, u8 segment, u8 mode)
{
  assert_param(digit_id < LED_DIGIT_COUNT && segment < LED_SEGMENT_COUNT);
  LED_TUBE* tube = &LED_DIGIT[digit_id][segment];
  gpio_write(tube->gpio, (BitAction) mode);
}

void game_counter_colon_set(u8 mode)
{
  gpio_write(LED_COLON.gpio, (BitAction)mode);
}

void game_counter_set_digit_id(u8 digit_id, u8 digit)
{
  assert_param(digit_id < LED_DIGIT_COUNT && digit <= 10);
  for (u8 i = 0; i < LED_SEGMENT_COUNT; ++i) {
    game_counter_tube_set(digit_id, i, DIGIT_MAP[digit][i]);
  }
}

void game_counter_set_time(u8 minute, u8 seconds)
{
  while (seconds > 60) {
    ++minute;
    seconds -= 60;
  }
  
  game_counter_set_digit_id(0, minute  % 10);
  game_counter_set_digit_id(1, (seconds / 10) % 10);
  game_counter_set_digit_id(2, seconds % 10);
}


void game_counter_display_up(void)
{
  game_counter_set_digit_id(0, 10);
  game_counter_set_digit_id(1, 11);
  game_counter_set_digit_id(2, 12);
}
