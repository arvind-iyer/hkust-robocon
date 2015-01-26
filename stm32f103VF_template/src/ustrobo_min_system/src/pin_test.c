#include "pin_test.h"

void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}
GPIO_Input set_init(u16 A, u16 B, u16 C, u16 D, u16 E)
{
  GPIO_Input GPIO_init_data = {A, B, C, D, E};
  return GPIO_init_data;
}

GPIO_Input sum(GPIO_Input input_data, u16 pin)
{
  if (pin <= 0xAF) {
    input_data.GPIOA_input |= pin;
  } else if (pin < 0xBF) {
    input_data.GPIOB_input |= pin;
  } else if (pin < 0xCF) {
    input_data.GPIOC_input |= pin;
  } else if (pin < 0xDF) {
    input_data.GPIOD_input |= pin;
  } else if (pin < 0xEF) {
    input_data.GPIOE_input |= pin;
  }
  return input_data;
}

bool equal_input(GPIO_Input current_val, GPIO_Input init_val)
{
  return (current_val.GPIOA_input == init_val.GPIOA_input && \
          current_val.GPIOB_input == init_val.GPIOB_input && \
          current_val.GPIOC_input == init_val.GPIOC_input && \
          current_val.GPIOD_input == init_val.GPIOD_input && \
          current_val.GPIOE_input == init_val.GPIOE_input);
}

void pin_test(void)
{
  // B0 is buzzer, C1 to C3 are led, D5 to D7 are lcd
  const u16 no_test_pin[] = {0xB0, 0xC1, 0xC2, 0xC3, 0xD5, 0xD6, 0xD7};
  u32 current_pin = 0xA0;
  static u16 ticks_img = 65535;
  bool test_finished = false;
  GPIO_Input GPIO_init_data = set_init(GPIO_ReadInputData(GPIOA), GPIO_ReadInputData(GPIOB) | 0xB0,
                                       GPIO_ReadInputData(GPIOC) | 0xC1 | 0xC2 | 0xC3,
                                      GPIO_ReadInputData(GPIOD) | 0xD5 | 0xD6 | 0xD7, GPIO_ReadInputData(GPIOE));
  GPIO_Input curr_input = GPIO_init_data;
  GPIO_Input last_input = GPIO_init_data;
  while (true) {
    if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
      if (ticks_img % 50 == 3) {
        curr_input = set_init(GPIO_ReadInputData(GPIOA), GPIO_ReadInputData(GPIOB) | 0xB0,
                              GPIO_ReadInputData(GPIOC) | 0xC1 | 0xC2 | 0xC3,
                              GPIO_ReadInputData(GPIOD) | 0xD5 | 0xD6 | 0xD7, GPIO_ReadInputData(GPIOE));      
        if (equal_input(last_input,GPIO_init_data) && equal_input(curr_input, sum(GPIO_init_data, current_pin))) {
//          SUCCESSFUL_MUSIC;
          ++current_pin;
        } else if (equal_input(last_input,GPIO_init_data) && !equal_input(curr_input, GPIO_init_data)) {
          FAIL_MUSIC;
        }
        last_input = curr_input;
      }
       
			if (ticks_img % 50 == 6) {
        tft_clear();
        draw_top_bar();
        tft_prints(0, 1, "Pin test");
        tft_prints(0, 2, "Plug %1X%d", current_pin / 0x10, current_pin % 0x10);
        tft_update();
      }
      
      
    }
  }
}
