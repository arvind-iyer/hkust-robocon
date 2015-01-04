#include "main.h"

u16 ticks_img = (u16) -1;			// Trival value
u16 seconds_img = (u16) -1;		// Trival value




int main(void)
{
	/*** Your code ***/

	ticks_init();
	tft_init(0, YELLOW, WHITE, RED);
	buzzer_init();
	led_init();
	
	
	/*
	uart_init(COM1, 115200);
	uart_interrupt(COM1);
	uart_init(COM2, 115200);
	uart_interrupt(COM2);
	uart_init(COM3, 115200);
	uart_interrupt(COM3);
	uart_init(COM4, 115200);
	uart_interrupt(COM4);
	uart_init(COM5, 115200);
	uart_interrupt(COM5);
	*/
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	buzzer_control(1, 200);
	_delay_ms(1000);
	buzzer_control(2, 100);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	
	
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 500 == 0) {
				//buzzer_control(3, 100);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), (LED_STATE) (ticks_img == 0));
//				uart_tx(COM1, "COM1 Output: %d\r\n", get_seconds());
//				uart_tx(COM2, "COM2 Output: %d\r\n", get_seconds());
//				uart_tx(COM3, "COM3 Output: %d\r\n", get_seconds());
//				uart_tx(COM4, "COM4 Output: %d\r\n", get_seconds());
//				uart_tx(COM5, "COM5 Output: %d\r\n", get_seconds());
				
				tft_clear();
				tft_prints(0, 0, "HELLO_WORLD!");
				tft_prints(0, 1, "Seconds: %d", get_seconds());
				tft_update();
				
			}
		}
		
		
	}

}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		u8 rx_data = (u8)USART_ReceiveData(USART1);
		uart_tx_byte(COM1, rx_data);
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		u8 rx_data = (u8)USART_ReceiveData(USART2);
		uart_tx_byte(COM2, rx_data);
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		u8 rx_data = (u8)USART_ReceiveData(USART3);
		uart_tx_byte(COM3, rx_data);
	}
}

void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		u8 rx_data = (u8)USART_ReceiveData(UART4);
		uart_tx_byte(COM4, rx_data);
	}
}

void UART5_IRQHandler(void)
{
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		u8 rx_data = (u8)USART_ReceiveData(UART5);
		uart_tx_byte(COM5, rx_data);
	}
}

