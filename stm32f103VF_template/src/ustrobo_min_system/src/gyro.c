#include "gyro.h"
#include "approx_math.h"

static POSITION gyro_pos = {0, 0, 0};

static u8 rx_state = 0; 
static u8 rx_command = 0;
static u8 buf_rec = 0;
static u8 buf_data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 rx_command_arr[GYRO_COMMAND_LENGTH] = {GYRO_UPDATED, GYRO_REPLY};
static u8 buf_len[GYRO_COMMAND_LENGTH] = {0x06, 0x01};		//data size, for confirm data

volatile u8 reply_flag = 0;

volatile u8 gyro_available = 0;

/**
  * @brief  Initialization of Gyro
  * @param  None
  * @retval None
  */
void gyro_init(void)
{
	uart_init(GYRO_UART, 115200);
	uart_interrupt(GYRO_UART);
}

s16    SHIFT_X = 0; // -200;	//53//	193//  -163	//	98		//79
s16    SHIFT_Y = 0; // -618;	//40// 	-50// 	-41	//	170		//336

/**
	* @brief Get the position object
	* @param None
	* @retval The position object
	*/
const POSITION* get_pos(void)
{
	return &gyro_pos;
}

/**
  * @brief  Get the X coordinate
  * @param  None
  * @retval X coordinate
  */
s16 get_X(void)
{
	
  s16 pos_x = (X_FLIP*gyro_pos.x*10000-SHIFT_X*10000+SHIFT_X*int_cos(gyro_pos.angle)+SHIFT_Y*int_sin(gyro_pos.angle))/10000;
	return pos_x;
}

/**
  * @brief  Get the Y coordinate
  * @param  None
  * @retval Y coordinate
  */
s16 get_Y(void)
{
	
	s16 pos_y = (Y_FLIP*gyro_pos.y*10000-SHIFT_Y*10000+SHIFT_Y*int_cos(gyro_pos.angle)-SHIFT_X*int_sin(gyro_pos.angle))/10000;
	return pos_y;
}

/**
  * @brief  Get the Angle (yaw)
  * @param  None
  * @retval Angle (yaw)
  */
s16 get_angle(void)
{
	return gyro_pos.angle;
}

/**
  * @brief  Send the update flag
  * @param  None
  * @retval None
  */
void gyro_pos_update(void)			// unuseful
{
	uart_tx_byte(GYRO_UART, GYRO_WAKEUP);
	uart_tx_byte(GYRO_UART, GYRO_UPDATE);
	uart_tx_byte(GYRO_UART, 0);
}

/**
  * @brief  Send the calibration flag
  * @param  None
  * @retval 1 = successful, 0 = failed
  */
u8 gyro_cal(void)
{
	u16 ticks_last = get_ticks();
	reply_flag &= ~GYRO_FLAG_CAL;
	
	uart_tx_byte(GYRO_UART, GYRO_WAKEUP);
	uart_tx_byte(GYRO_UART, GYRO_CAL);
	uart_tx_byte(GYRO_UART, 0);
	
	while (!(reply_flag & GYRO_FLAG_CAL)) {
		if ((get_ticks()+1000-ticks_last) % 1000 >= 20)			// 20 ms timeout
			return 0;
	}
	return 1;
}

/**
  * @brief  Update the offset of X, Y coordinate and Angle
  * @param  x: X coordinate to be set
  * @param  y: Y coordinate to be set
  * @param  a: angle to be set
  * @retval 1 = successful, 0 = failed
  */
u8 gyro_pos_set(s16 x, s16 y, s16 a)
{
	u16 ticks_last = get_ticks();
	reply_flag &= ~GYRO_FLAG_SET_POS;
	// Shift
	x = (x*10000+SHIFT_X*10000-SHIFT_X*int_cos(a)-SHIFT_Y*int_sin(a))/10000;
	y = (y*10000+SHIFT_Y*10000-SHIFT_Y*int_cos(a)+SHIFT_X*int_sin(a))/10000;
	
	
	uart_tx_byte(GYRO_UART, GYRO_WAKEUP);
	uart_tx_byte(GYRO_UART, GYRO_POS_SET);
	uart_tx_byte(GYRO_UART, 0x06);
	uart_tx_byte(GYRO_UART, x >> 8);
	uart_tx_byte(GYRO_UART, x & 0xFF);
	uart_tx_byte(GYRO_UART, y >> 8);
	uart_tx_byte(GYRO_UART, y & 0xFF);
	uart_tx_byte(GYRO_UART, a >> 8);
	uart_tx_byte(GYRO_UART, a & 0xFF);
	
	while (!(reply_flag & GYRO_FLAG_SET_POS)) {
		if ((get_ticks()+1000-ticks_last) % 1000 >= 20)			// 20 ms timeout
			return 0;
	}

	return 1;
}

/**
  * @brief  Interrupt for USART3
  * @param  None
  * @retval None
  */


void USART3_IRQHandler(void)
{
	u8 rx_data, i;
	u16 x, y, a;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		rx_data = (u8)USART_ReceiveData(USART3);
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
				if (rx_command == 0xFF)	// command not in list		
					rx_state = 0;
				break;
			case 2: // confirm command
				if (rx_data != buf_len[rx_command]) {		// wrong data length
					rx_state = 0;
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
				switch (rx_command) {
					case 0:		// GYRO_UPDATED
						x = buf_data[0];
						x <<= 8;
						x |= buf_data[1];
						y = buf_data[2];
						y <<= 8;
						y |= buf_data[3];
						a = buf_data[4];
						a <<= 8;
						a |= buf_data[5];
						
						if (a < 3600) {
							gyro_available = 1;
							
							gyro_pos.x = (s16) x;
							gyro_pos.y = (s16) y;
							gyro_pos.angle = (s16) a;
						} else {
							gyro_available = 0;
						}
						break;
					case 1:		// GYRO_REPLY
						reply_flag |= (1 << buf_data[0]);
						break;
							
				}
				rx_state = 0;
				break;
		}
		
	}
	
	//USART_ClearFlag(USART3,USART_FLAG_RXNE);
	//USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}


