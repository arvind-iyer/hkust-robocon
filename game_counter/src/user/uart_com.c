#include "uart_com.h"

bool uart_set_flag = false; 
u16 timer_set_counter = 0;
u16 buffer_i = 0;
u8 tmp_digit[3] = {0,0,0};
char uart_buffer[UART_BUFFER_SIZE+1] = {0};
u16 uart_buffer_i = 0;

static void uart_buffer_reset(void)
{
  uart_buffer[0] = '\0';
  uart_buffer_i = 0;
  timer_set_counter = 0;
  timer_clock_set_flag(TIMER_SET_OFF);
  uart_set_flag = false;
}

static void uart_com_handler(u8 data)
{
  if (get_timer_mode()) {
    uart_set_flag = true;
    uart_buffer[uart_buffer_i % UART_BUFFER_SIZE] = (char)data;
    uart_buffer[uart_buffer_i % UART_BUFFER_SIZE + 1] = '\0';
    uart_buffer_i = (uart_buffer_i + 1) % UART_BUFFER_SIZE;
    if (strstr(uart_buffer, "timer")) {
      timer_clock_mode_toggle(false);
      uart_buffer_reset();
    } else if (strstr(uart_buffer, "get")) {
      
      //uart_com_tx();
      u32 current_time = get_current_time();
      
      u8 second = current_time % 60;
      u8 minute = (current_time / 60) % 60;
      u8 hour = (current_time / 3600) % 24;
      uart_com_tx("%02d:%02d:%02d\r\n", hour, minute, second); 
      
      for (u8 i = 0; i < ALARM_COUNT; ++i) {
        const ALARM* alarm = get_alarm(i);
        if (alarm->flag) {
          uart_com_tx("Alarm%d: %02d:%02d\r\n", i, (alarm->time / 3600) % 24, (alarm->time / 60) % 60);
        }
      }    
      uart_buffer_reset();
      
      buzzer_control_note(2, 100, NOTE_C, 7);
        
    } else if (strstr(uart_buffer, "volume0")) {
      uart_buffer_reset();
      buzzer_set_volume(2);
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "volume1")) {
      uart_buffer_reset();
      buzzer_set_volume(10);
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "volume2")) {
      uart_buffer_reset();
      buzzer_set_volume(20);
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "volume3")) {
      uart_buffer_reset();
      buzzer_set_volume(30);
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "volume4")) {
      uart_buffer_reset();
      buzzer_set_volume(40);
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "set_hour")) {
      timer_set_counter = 0;
      uart_buffer[0] = '\0';
      timer_clock_set_flag(TIMER_SET_HOUR);
      buzzer_control_note(3, 100, NOTE_C, 7);
    } else if (strstr(uart_buffer, "set_minute")) {
      timer_set_counter = 0;
      uart_buffer[0] = '\0';
      timer_clock_set_flag(TIMER_SET_MINUTE);
      buzzer_control_note(3, 100, NOTE_C, 7);
    } else if (strstr(uart_buffer, "set_second")) {
      uart_buffer_reset();
      u32 time = get_current_time();
      time = (time / 60) * 60;
      set_current_time(time);
      buzzer_control_note(3, 100, NOTE_C, 7);
    } else if (strstr(uart_buffer, "mario_start")) {
      uart_buffer_reset();
      MARIO_BEGIN_MUSIC;
    } else if (strstr(uart_buffer, "mario_song")) {
      uart_buffer_reset();
      MARIO_MUSIC;
    } else if (strstr(uart_buffer, "mario_end")) {
      uart_buffer_reset();
      MARIO_END_MUSIC;
    } else if (strstr(uart_buffer, "birthday")) {
      uart_buffer_reset();
      BIRTHDAY_MUSIC;
    } else if (uart_buffer_i >= 2) {
      // Check alarm
       for (u8 i = 0; i < ALARM_COUNT; ++i) {
        char buffer[50] = "alarm", buffer2[50];
        char i_str[5];
        sprintf(i_str, "%d", i);
        strcat(buffer, i_str);
        
          {
            // On
            strcpy(buffer2, buffer);
            strcat(buffer2, "_on");
            //uart_com_tx("%s\r\n", uart_buffer_tmp);
            //uart_com_tx("%s (%d)\r\n", buffer2, strstr(uart_buffer_tmp, buffer2));
            if (strstr(uart_buffer, buffer2)) {
              uart_buffer_reset();
              set_alarm_flag(i, true);
              buzzer_control_note(3, 100, NOTE_C, 8);
              return;
            }
          }
          
          {
            // Off
            strcpy(buffer2, buffer);
            strcat(buffer2, "_off");
            if (strstr(uart_buffer, buffer2)) {
              uart_buffer_reset();
              set_alarm_flag(i, true);
              buzzer_control_note(3, 100, NOTE_C, 8);
              return;
            }
          }
        
          {
            // Set_minute
            strcpy(buffer2, buffer);
            strcat(buffer2, "_minute");
            if (strstr(uart_buffer, buffer2)) { 
              timer_set_counter = 0;
              uart_buffer[0] = '\0';
              timer_clock_set_flag(TIMER_SET_ALARM_MINUTE);
              alarm_id_tmp_set(i);
              buzzer_control_note(3, 100, NOTE_C, 7);
              return;
            }
          }

          {
            // Set_hour
            strcpy(buffer2, buffer);
            strcat(buffer2, "_hour");
            strcpy(uart_buffer, uart_buffer);
            if (strstr(uart_buffer, buffer2)) { 
              timer_set_counter = 0;
              uart_buffer[0] = '\0';
              timer_clock_set_flag(TIMER_SET_ALARM_HOUR);
              alarm_id_tmp_set(i);
              buzzer_control_note(3, 100, NOTE_C, 7);
              return;
            }
          }        
        }
        
      // Input the latest TWO digits ASCII
      if (uart_buffer[uart_buffer_i - 1] >= '0' && uart_buffer[uart_buffer_i - 1] <= '9' && \
        uart_buffer[uart_buffer_i - 2] >= '0' && uart_buffer[uart_buffer_i - 2] <= '9') {
          u8 input = (uart_buffer[uart_buffer_i - 2] - '0') * 10 + uart_buffer[uart_buffer_i - 1] - '0';
          if (timer_clock_set(input)) {
            uart_buffer_reset();
            timer_clock_set_flag(TIMER_SET_OFF);
            buzzer_control_note(3, 100, NOTE_C, 8);
          } else {
            return;
          }
        }

    } else {
      return;
    }

    
  } else {
    if (data >= '0' && data <= '9') {
      u8 digit = data - '0';
      uart_set_flag = true;
      switch (buffer_i % 3) {
        case 0:
          tmp_digit[2] = digit;
          buzzer_control_note(1, 80, NOTE_E, 6);
        break;
        
        case 1:
          tmp_digit[1] = tmp_digit[2];
          tmp_digit[2] = digit; 
          buzzer_control_note(1, 80, NOTE_G, 6);
        break;
        
        case 2:
          tmp_digit[0] = tmp_digit[1];
          tmp_digit[1] = tmp_digit[2];
          tmp_digit[2] = digit;
          buzzer_control_note(3, 80, NOTE_C, 7);
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
      
      if (!is_timer_start()) {
        timer_set(60);
        timer_next_set(5);
        buzzer_control_note(2, 80, NOTE_D, 7); 
      }
    } else if (data == 's' || data == 'S') {
      
      if (!is_timer_start()) {
        timer_set(5);
        timer_next_set(0);
        buzzer_control_note(2, 80, NOTE_E, 7);
      }
      
    } else if (data == 'r' || data == 'R') {
      if (!is_timer_start()) {
        timer_set(15);
        timer_next_set(5);
        buzzer_control_note(2, 80, NOTE_Fs, 7);
      }
    } else if (data == 't' || data == 'T') {
      
      if (!is_timer_start()) {
        timer_set(30);
        timer_next_set(5);
        buzzer_control_note(2, 80, NOTE_G, 7);
      }
    } else if (data == 'g' || data == 'G') {
      u32 timer_ms = get_timer_ms();
      buzzer_control_note(2, 80, NOTE_C, 7);
      uart_com_tx("%02d:%02d.%03d\r\n", (timer_ms / 1000) / 60, (timer_ms / 1000) % 60, timer_ms % 1000); 
    } else if (data == 'u' || data == 'U') {
      timer_set(0);
      if (!is_timer_start()) {
        buzzer_control_note(2, 80, NOTE_A, 7);
      }
    } else if (data == 'c' || data == 'C') {
      timer_clock_mode_toggle(true); 
      
      
    }
  }
}

void uart_com_init(void)
{
  uart_init(UART_COM, UART_BR);
  uart_rx_init(UART_COM, uart_com_handler);
  buffer_i = 0;
  tmp_digit[0] = tmp_digit[1] = tmp_digit[2] = 0;
  uart_buffer_i = 0;
  uart_buffer[0] = '\0'; 
}

void uart_com_tx(const char * tx_buf, ...)
{
	va_list arglist;
	u8 buf[40], *fp;
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte(UART_COM,*fp++);
}

void uart_update(void)
{
  if (uart_set_flag) {
    ++timer_set_counter;
    if (timer_set_counter >= 1000) {  // 10 seconds
      timer_set_counter = 0;
      buffer_i = 0;
      uart_buffer_i = 0;
      uart_set_flag = false;
      tmp_digit[0] = tmp_digit[1] = tmp_digit[2] = 0;
      timer_clock_set_flag(TIMER_SET_OFF);
    }
  }
  
}
