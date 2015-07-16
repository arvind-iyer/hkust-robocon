#include "gpio.h"

#define small_button   &PE3
#define up_button   &PE5
#define center_button   &PC14
#define left_button   &PE4
#define right_button   &PC13
#define down_button   &PC15

void button_init(void);
u8 button_read(const GPIO* gpio);
