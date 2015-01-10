#include "light_sensor.h"
#include "approx_math.h"

void ls_init(void)
{
	uart_init(LS_UART, 115200);
	uart_interrupt(LS_UART);
}

u8 get_ls(void) {
	return USART_ReceiveData(USART3);
}

u8 test_ls(s8 top_left, s8 top_right, s8 bottom_left, s8 bottom_right) {
	s8 i, j, case0, case1, x[4];

	x[0] = top_left;
	x[1] = top_right; 
	x[2] = bottom_left;
	x[3] = bottom_right;
	
	for (i = 0; i < 4; i++) {
		if (x[i] == X) {
			x[i] = 0;
			case0 = test_ls(x[0], x[1], x[2], x[3]);
			x[i] = 1;
			case1 = test_ls(x[0], x[1], x[2], x[3]);
			return (case0 || case1);
		}
	}
	return (get_ls() == LS_TL * top_left + LS_TR * top_right + LS_BL * bottom_left + LS_BR * bottom_right);
};
