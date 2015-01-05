#include "main.h"

u16 ticks_img = (u16) -1;			// Trival value
u16 seconds_img = (u16) -1;		// Trival value



void but_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}


int main(void)
{
	/*** Your code ***/
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM6|RCC_APB1Periph_TIM7|RCC_APB1Periph_USART3, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |  RCC_APB2Periph_TIM1 |RCC_APB2Periph_TIM8| RCC_APB2Periph_USART1,ENABLE);


	ticks_init();
	tft_init(1, YELLOW, RED, GREEN);	
	buzzer_init();
	led_init();
	but_init();
	
	buzzer_play_song(START_UP, 100, 10);

	
	
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	_delay_ms(1000);
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
				
			
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0) {
					buzzer_control(2, 100);
				}
				
				if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0) {
					buzzer_control(3, 100);
				}
				
			}

		}
		
		//GPIO_WriteBit(GPIOD, GPIO_Pin_5, GPIO_ReadOutputDataBit(GPIO_CS, GPIO_Pin_CS));
		
		
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

