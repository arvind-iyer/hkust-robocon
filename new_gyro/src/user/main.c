#include "main.h"

//u8 rx_state = 0;
//u8 rx_command = 0;
//u8 buf_rec = 0;

u8 rx_command_arr[GYRO_COMMAND_LENGTH] = {GYRO_UPDATE, GYRO_CAL, GYRO_POS_SET, GYRO_AUTO_UPDATE};
u8 buf_len[GYRO_COMMAND_LENGTH] = {0x00, 0x00, 0x06, 0x00};		//data size, for confirm data

u8 gyro_auto_update = 1;		// auto_update == 1 , position will send automatically 

u8 uart_buf[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
u8 uart_buf_rec = 0;
u16 uart_cnt = 0;

int main(void)
{
	u16 ticks_img = 0;
	
	// clock setting
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	
	uart_init(COM1, 115200);
	uart_printf_enable(COM1);
	
	gyro_init();
	ticks_init();
	buzzer_init();
	
	printf("gyro init ...");
	printf("done\r\nencoder init ...");
	encoder_init();
	printf("done\r\n");
	
	buzzer_control(2, 5);
	
	gyro_cal();
    
	buzzer_control(1, 30);
	
	uart_init(COM3, 115200);
	uart_interrupt(COM3);
	while (1)
	{
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 128 == 0) {  		// ~64ms
				//printf("%d %d %d x\r\n", real_X , real_Y , real_angle);
				printf("X: %d , Y: %d , A: %d, I: %d \r\n" , real_X , real_Y , real_angle , gyro_angle );
				//printf( "E0:%ld , E1:%ld , E2:%ld \r\n" , read_encoder(0) , read_encoder(1)  , read_encoder(2));
			}
			if (uart_cnt > 0) {
				if (--uart_cnt == 0) {
					uart_update();
				}
			}
			if (gyro_auto_update) {
				if (ticks_img % 4 == 2) {	//~4ms
					send_pos();
				}
			}
		}
	}

}

void send_pos(void)
{
	s16 x = real_X;
	s16 y = real_Y;
	s16 a = real_angle;
	if (gyro_state != 2) {
		a = 0x7FFF;		// not available
	}
	uart_tx_byte(GYRO_COM, GYRO_WAKEUP);
	uart_tx_byte(GYRO_COM, GYRO_UPDATED);
	uart_tx_byte(GYRO_COM, 6);				//data length
	uart_tx_byte(GYRO_COM, x >> 8);
	uart_tx_byte(GYRO_COM, x & 0xFF);
	uart_tx_byte(GYRO_COM, y >> 8);
	uart_tx_byte(GYRO_COM, y & 0xFF);
	uart_tx_byte(GYRO_COM, a >> 8);
	uart_tx_byte(GYRO_COM, a & 0xFF);
}

void send_reply(u8 data)
{
	uart_tx_byte(GYRO_COM, GYRO_WAKEUP);
	uart_tx_byte(GYRO_COM, GYRO_REPLY);
	uart_tx_byte(GYRO_COM, 1);
	uart_tx_byte(GYRO_COM, data);
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		if (uart_buf_rec >= 10)
			return;
		uart_buf[uart_buf_rec] = (u8)USART_ReceiveData(USART3);
		uart_buf_rec++;
		uart_cnt = 5;
		gyro_state = 1;
	}
	
	//USART_ClearFlag(USART3,USART_FLAG_RXNE);
	//USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}

void uart_update(void)
{
	u8 i, k, rx_data;
	u16 x, y, a;
	u8 rx_state = 0;
	u8 rx_command = 0;
	u8 buf_rec = 0;
	u8 buf_data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	uart_buf_rec = 0;
	if (uart_buf[0] != GYRO_WAKEUP) {
		gyro_state = 2;
		return;
	}
	
	for (k = 0; k < 10; k++) {
		rx_data = uart_buf[k];
		uart_buf[k] = 0;
		
		switch (rx_state) {
			case 0:	// wakeup
				if (rx_data == GYRO_WAKEUP) {
					rx_command = 0xFF;
					buf_rec = 0;
					rx_state++;
				}
				break;
			case 1:	// command
				for (i = 0; i < GYRO_COMMAND_LENGTH; i ++) {
					if (rx_data == rx_command_arr[i]) {
						rx_command = i;
						rx_state++;
						break;
					}
				}
				if (rx_command == 0xFF) {	// command not in list		
					rx_state = 0;
					gyro_state = 2;
				}
				break;
			case 2: // confirm command
				if (rx_data != buf_len[rx_command]) {		// wrong data length
					rx_state = 0;
					gyro_state = 2;
					break;
				}
				rx_state++;
				if (buf_len[rx_command] > 0) {
					break;
				}
			case 3: // receive data
				if (buf_len[rx_command] == 0) {
					rx_state++;
				} else {
					buf_data[buf_rec++] = rx_data;
					if (buf_rec >= buf_len[rx_command]) {
						rx_state++;
					} else {
						break;
					}
				}
			case 4:
				buzzer_control(1, 3);
				switch (rx_command) {
					case 0:
						send_pos();
						break;
					case 1:
						printf("short cal\r\n");
						gyro_cal_short();
						send_reply(GYRO_REPLAY_CAL);
						buzzer_control(1, 30);
						break;
					case 2:
						x = buf_data[0];
						x <<= 8;
						x |= buf_data[1];
						y = buf_data[2];
						y <<= 8;
						y |= buf_data[3];
						a = buf_data[4];
						a <<= 8;
						a |= buf_data[5];
						printf("pos_set (%d, %d, %d)\r\n", (s16)x, (s16)y, (s16)a);
						send_reply(GYRO_REPLAY_SET_POS);
						position_set((s16)x, (s16)y, (s16)a);
						buzzer_control(3, 5);
						break;
					case 3:
						printf("auto_update (%d)\r\n", buf_data[0]);
						gyro_auto_update = buf_data[0];
						break;
				}
				rx_state = 0;
				gyro_state = 2;
				return;
		}
		if (gyro_state == 2) {
			return;
		}
	}
}
