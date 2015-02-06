#include "uart_com.h"

bool timer_set_flag = false; 
u16 timer_set_counter = 0;
u16 buffer_i = 0;
u8 tmp_digit[3] = {0,0,0};

static void uart_com_handler(u8 data)
{
  if (data >= '0' && data <= '9') {
    u8 digit = data - '0';
    timer_set_flag = true;
    switch (buffer_i % 3) {
      case 0:
        tmp_digit[2] = digit;
        buzzer_control(1, 80);
      break;
      
      case 1:
        tmp_digit[1] = tmp_digit[2];
        tmp_digit[2] = digit; 
        buzzer_control(1, 80);
      break;
      
      case 2:
        tmp_digit[0] = tmp_digit[1];
        tmp_digit[1] = tmp_digit[2];
        tmp_digit[2] = digit;
        buzzer_control(3, 50);
      break;
    }
    
    buffer_i = (buffer_i + 1) % 3;
    timer_set(tmp_digit[0] * 60 + tmp_digit[1] * 10 + tmp_digit[2]);
  } else if (data == ' ') {
    timer_start(3);
  } else if (data == '\n') {
    timer_start(0);
  } else if (data == 27 || data == 'q' || data == 'Q') {
    timer_stop();
  } else if (data == 'p' || data == 'P') {
    timer_set(60);
    buzzer_control(2, 80);
  } else if (data == 's' || data == 'S') {
    timer_set(5);
    buzzer_control(2, 80);
  } else if (data == 'r' || data == 'R') {
    timer_set(15);
    buzzer_control(2, 80);
  } else if (data == 't' || data == 'T') {
    timer_set(30);
    buzzer_control(2, 80);
  }
}

void uart_com_init(void)
{
  uart_init(UART_COM, UART_BR);
  uart_rx_init(UART_COM, uart_com_handler);
  buffer_i = 0;
  tmp_digit[0] = tmp_digit[1] = tmp_digit[2] = 0;
}

void uart_update(void)
{
  if (timer_set_flag) {
    ++timer_set_counter;
    if (timer_set_counter == 300) {
      timer_set_counter = 0;
      buffer_i = 0;
      timer_set_flag = false;
      tmp_digit[0] = tmp_digit[1] = tmp_digit[2] = 0;
    }
  }
  
}
