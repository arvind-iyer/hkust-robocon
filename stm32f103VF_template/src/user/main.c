#include "main.h"

u16 ticks_img = (u16) -1;			// Trival value
u16 seconds_img = (u16) -1;		// Trival value


u16 hello = 0;
void but_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void can_rx_test(CanRxMsg msg)
{
	u16 id = msg.IDE;
	//buzzer_set_note_period(get_note_period(NOTE_A, 5));
	//buzzer_control(1, 20);
	
	++hello;
	//tft_prints(0, 6, "CAN test: %d", msg.FMI);
}


int main(void)
{
	/*** Your code ***/
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM6|RCC_APB1Periph_TIM7|RCC_APB1Periph_USART3, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |  RCC_APB2Periph_TIM1 |RCC_APB2Periph_TIM8| RCC_APB2Periph_USART1,ENABLE);


	ticks_init();
	tft_init(1, YELLOW, RED, GREEN);	
	buzzer_init();
	can_init();
	can_rx_init();
	
	
	//can_rx_add_filter(0x0A1, 0x7FF, can_rx_test);
	//can_rx_add_filter(0x0B0, 0x7F0, can_rx_test2);
	led_init();
	but_init();
	
	uart_init(COM1, 115200);
	
	buzzer_play_song(START_UP, 120, 0);
	
	
	//buzzer_control(2, 100);
	
	
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_ON);
	_delay_ms(1000);
	led_control((LED) (LED_D1 | LED_D2 | LED_D3), LED_OFF);
	

	
	while(1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 5 == 0 && get_seconds() > 0) {
				CAN_MESSAGE msg;
				msg.id = 0xA1;
				msg.length = 1;
				//msg.data[0] = (u8) ticks_img;
				msg.data[0] = (u8) get_seconds();
				can_tx_enqueue(msg);
			}

			if (ticks_img % 500 == 0) {
				//buzzer_control(3, 100);
				led_control((LED) (LED_D1 | LED_D2 | LED_D3), (LED_STATE) (ticks_img == 0));
				
			}
			
			if (ticks_img % 50 == 0) {
				tft_clear();
				tft_prints(0, 0, "HELLO_WORLD!");
				tft_prints(0, 1, "%d", hello);
				tft_update();
//				tft_prints(0, 1, "Seconds: %d", get_seconds());
//				tft_prints(0, 2, "Ticks: %d", ticks_img);
//				tft_prints(0, 3, "CAN Queue: %d", can_tx_queue_size());
//				tft_prints(0, 4, "(%3d - %3d)", can_tx_queue_head(), can_tx_queue_tail());
//				tft_prints(0, 5, "CAN Empty: %d", can_empty_mailbox());
//				tft_update();
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

