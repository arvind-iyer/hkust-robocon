#include "main.h"

ErrorStatus HSEStartUpStatus;

#define SW_RESET_PORT		GPIOA
#define SW_RESET_MAINBOARD_PIN	GPIO_Pin_0
#define SW_RESET_PRESSED	18 //0000 0000 //search read_button

u16 sw_reset_count = 0;
u16 ticks_img = 0;
u8 ticks_1ms = 0;
u16 seconds_img = 0;
u8 volatile is_xbc_board_processing = 1; //1 yes, 0 no
u8 volatile is_spi_interrupting = 0; // 1 yes, 0 no

u8 volatile finish_init = 0;
/****/
// for test data!!!!!!

u16 volatile tick_in = 0;
u16 volatile tick_out = 0;

u16 volatile tick_in2 = 0;
u16 volatile tick_out2 = 0;

u16 volatile tick_in3 = 0;
u16 volatile tick_out3 = 0;

u8 volatile test_exti =0;
u8 volatile test_int_num = 0;
u8 volatile camble_flag = 0; // 1: change camble light
/****/

void show_xbcdata(){
s16 lx = 0;
s16 rx = 0;
s16 ly = 0;
s16 ry = 0;
u16 lt = 0;
u16 rt = 0;	
	
	//while(!set_xbc_config(2)); /* turn one rumble */
	
	if (xbc_data[0] == 0x14 && !is_failure_data)
	{	
	//	printf("\n\r %xH \n\r",check_ch376_conn_statusstatus());
		if (xbc_data[3] == 0 && xbc_data[4] == 0)
			tft_prints(0, 2, "            ");
		if (xbc_data[3] != 0)
		{	
			
			switch(xbc_data[3])
			{			
				
				case 0x01:
					tft_prints(0, 2, "DPAD UP   ");
				//	while(!set_xbc_config(1)); /* turn one rumble */
					break;
				case 0x02:
					tft_prints(0, 2, "DPAD DOWN ");
				//	while(!set_xbc_config(2)); /* turn one rumble */
					break;
				case 0x04:
					tft_prints(0, 2, "DPAD LEFT ");
				//	while(!set_xbc_config(3)); /* turn one rumble */
					break;
				case 0x08:
					tft_prints(0, 2, "DPAD RIGHT");
				//	while(!set_xbc_config(4)); /* turn one rumble */
					break;
				case 0x10:
					tft_prints(0, 2, "START     ");
					break;
				case 0x20:
					tft_prints(0, 2, "BACK      ");
					break;
				default:
					break; 
			}
		}
		if (xbc_data[4] != 0)
		{
			switch(xbc_data[4])
			{
				case 0x01:
					tft_prints(0, 2, "LB        ");
					break;
				case 0x02:
					tft_prints(0, 2, "RB        ");
					break;
				case 0x04:
					tft_prints(0, 2, "XBOX      ");
					break;
				case 0x10:
					tft_prints(0, 2, "A         ");
				//	while(!set_xbc_config(5)); /* turn one rumble */
					break;
				case 0x20:
					tft_prints(0, 2, "B         ");
				//	while(!set_xbc_config(6)); /* turn one rumble */
					break;
				case 0x40:
					tft_prints(0, 2, "X         ");
					break;
				case 0x80:
					tft_prints(0, 2, "Y         ");
					break;
				default:
					break; 
			}
		}
		lx = (xbc_data[8]<<8) + xbc_data[7];
		ly = (xbc_data[10]<<8) + xbc_data[9];
		rx = (xbc_data[12]<<8) + xbc_data[11];
		ry = (xbc_data[14]<<8) + xbc_data[13];
		lt = xbc_data[5];
		rt = xbc_data[6];
		tft_prints(0, 3, "LX = %6d", lx);
		tft_prints(0, 4, "LY = %6d", ly);
		tft_prints(0, 5, "RX = %6d", rx);
		tft_prints(0, 6, "RY = %6d", ry);
		tft_prints(0, 7, "LT = %3d", lt);
		tft_prints(0, 8, "RT = %3d", rt);		
	}
	else{
		tft_prints(0, 2, "    fail    ");
	}
	
	tft_prints(15, 9, "%d", get_seconds() % 10);
	
	tick_in = get_ticks();
	tft_update();
	tick_out = get_ticks();
	
	printf("i:%d,o:%d", tick_in,tick_out);
}

void show_fromuart(void){
u8 i = 0;
	if (get_usb_state() != USB_STATE_TKN_X && get_usb_state() == USB_STATE_CONFIGURED){// && !is_failure_data){
		//printf("\r\nOK\r\n");
		//printf("[%2X] ",returntmpss());
		printf("[%xH:]\r\r",read_tkn_status());
		for(i=0;i<20;i++){
			printf("%2X ",xbc_data[i]);
		}					
		printf("\r\n");
	
	}

}

void read_button(void){
	
	tx_xbc_data[13] = 0;
	
	if (!GPIO_ReadInputDataBit(GPIOC,P0))
		tx_xbc_data[13] = tx_xbc_data[13] | 1;
		
	if (!GPIO_ReadInputDataBit(GPIOC,P1))
		tx_xbc_data[13] = tx_xbc_data[13] | 2;
	
	if (!GPIO_ReadInputDataBit(GPIOC,P7))
		tx_xbc_data[13] = tx_xbc_data[13] | 4;
		
	if (!GPIO_ReadInputDataBit(GPIOA,P2))
		tx_xbc_data[13] = tx_xbc_data[13] | 8;
	
	if (!GPIO_ReadInputDataBit(GPIOC,P5))
		tx_xbc_data[13] = tx_xbc_data[13] | 16;
		
	if (!GPIO_ReadInputDataBit(GPIOC,P6))
		tx_xbc_data[13] = tx_xbc_data[13] | 32;	
		
	
}

static void check_usb_connection(void)
{      
  static u8 disconnected = 0;
  while(usb_main_loop() != USB_STATE_CONFIGURED)
  {
    static u16 last_second = 999;
    //disconnected = 1;
     if (get_seconds() % 2 == 1 && (get_ticks() == 10 || get_seconds() != last_second && get_ticks() > 10)) {
       last_second = get_seconds();
       buzzer_control_note(2, 100, NOTE_C, 4);
     }
   }
  
   if (disconnected) {
     reset_all_pin(); 
     disconnected = 0;
   }
}

int main(void){
	
	u8 temp = 0;	
	u8 i = 0;
	u8 main_state = 0;
	u8 jump_state = 0;
	u16 delay_cnt = 450;	
	u8 volatile sent = 0;
	u8 xbc_current_state = 0;
	u8 xbc_time_out = 0; // 
	u8 spi_trig = 0;
	
	u8 phase = CH376_TKN_PHASE_DATA1;
	u8 buf_cnt = 0;
	u8 check_ch376_conn_status = 0;
	
	u16 check_bit_buf = 0;
	u8 check_first_8bit = 0;
	u8 check_last_8bit = 0;
	
/**************************************
 *init ch376 board and xbox controller*
 **************************************/	
	reset_all_pin(); //set all pin into floating, a trigger to reset ch376 & xbox controller
	enable_bt_printf(COM1);	
	tft_init(0, WHITE, BLACK, RED);
	ticks_init();
	buzzer_init();
  buzzer_set_note_period(get_note_period(NOTE_C, 5));
	buzzer_control_note(1,200, NOTE_C, 7);	
	tft_prints(0, 0, "STM32 Init OK!!...");
	tft_update();
	
	usb_init();
	usb_start_run();
	init_xbc_board();
	spi_init();
  xbc_chip_deselect();
	buzzer_control_note(2,60, NOTE_E, 7);
	
	while(!set_xbc_config(1)); /* turn on one rumble */	
	tft_clear();
	tft_prints(0, 0, "XBC Init OK!!...");
	tft_prints(0, 1, "SPI CONNECTING...");
	tft_update();
	finish_init = 1;
/****************
 *end of initial*
 ****************/
 

/*
 * Main program: tx & rx data between XBC & mainborad
 * 1.keep check if xbox controller is press
 * 2.
 */ 	
	while (1) {
	
	if (ticks_img != get_ticks()){
		ticks_img = get_ticks();
		
		switch(main_state){
			case 0 :			
				printf("1");
				xbc_time_out = 100;
				main_state = 1;
				jump_state = 1;
				
				CHECK_USB_BUFF(CH376_TKN_PHASE_DATA1);
				test_exti = 1;
			
			break;
			
			case 1:
        // TODO:
				//while(!check_ch376_usb_conn()); //checking usb connection	
        check_usb_connection();      
				is_failure_data = 1;
				
				while(!test_exti){
					check_ch376_conn_status = usb_get_status();
					
					if (check_ch376_conn_status != CH376_USB_INT_SUCCESS)
					{/* if error occur, leave current process, wait for next process */
						if (check_ch376_conn_status == 0x2B){
							phase &= 0x00;
						}
						else if (check_ch376_conn_status == 0x23){
							phase |= 0x80;
						}
						CHECK_USB_BUFF(phase);
						test_exti = 1;
						break;
					}
					/*
					 * first data is length of data block
					 * Second and Third data always 0x0 & 0x14
					 */
					read_usb_buff(); //read data block from current end point buffer
					for (buf_cnt=0;buf_cnt<20;buf_cnt++)
					{ 
						xbc_data[buf_cnt] = ch376_read_data();				
					}
					
					if (xbc_data[0] == 0x14 && xbc_data[1] == 0x0 && xbc_data[2] == 0x14)
					{						
						for(buf_cnt=1;buf_cnt<14;buf_cnt++)						
						{
							tx_xbc_data[buf_cnt] = xbc_data[buf_cnt+2];
							check_bit_buf += tx_xbc_data[buf_cnt];
						}
					
						check_bit_buf  = check_bit_buf;						
						
						tx_xbc_data[0] = 0x14; //start bit
						tx_xbc_data[14] = 0x15; //end bit					

						is_failure_data = 0;						
					}
					
					phase ^= USB_PHASING_MASK;
					CHECK_USB_BUFF(phase);
					test_exti = 1;
				}		
					main_state +=jump_state;				
			   		data_busy = 1;
					xbc_chip_select();
			
			break;	
			
			case 2:			
				
				read_button(); // read input button at bottom of controller
					
					/** reset mainboard function**/
					if (tx_xbc_data[13] == SW_RESET_PRESSED )
					{
						
						if (sw_reset_count++ > 500) /* 1tick 1 loop = 500ms*/
						{
							GPIO_SetBits(SW_RESET_PORT,SW_RESET_MAINBOARD_PIN);							
							buzzer_control(5,5);
							sw_reset_count = 0;
							_delay_ms(10);
							GPIO_ResetBits(SW_RESET_PORT,SW_RESET_MAINBOARD_PIN);	
						}
						
					}
					else 
					{
						sw_reset_count = 0;
					}
				  /************************************************/
					
				/* tx & rx process */
				ticks_1ms++;
				//printf("t:%d   b:%d\r\n",ticks_1ms, data_busy);
				
				if ( ticks_1ms > 1 && !data_busy ||ticks_1ms > 20 )
				{
					//printf("t:%d   b:%d\r\n",ticks_1ms, data_busy);
          for(u8 buf_cnt=1;buf_cnt<14;buf_cnt++)						
          {
            printf("%d ",tx_xbc_data[buf_cnt]);
          }
          printf("\r\n");
					ticks_1ms = 0;
					spi_trig = 0;
					xbc_chip_deselect();
					jump_state = main_state ; //next state == 3					
					main_state = 1;
					if (tft_receive_flag)
					{
						tft_set_all_color();
						tft_receive_flag = 0;
					}
				}
					
				//	show_xbcdata();
				
			break;
			
			case 3: //print tft				
				
				tft_update();
				jump_state = 1;
				main_state = 1;
				
				//jump_state = main_state ; //next state == 3					
				//main_state = 1;
			break;
			
			case 4:
		    
			break;
			
			default:
				main_state = 0;
				jump_state = 0;
			break;
						
		}	
	}
	}
}

void init_xbc_board(){
u8 exist;

	while(1)
	{	
			
	/* 
	 *check communication between mcu and ch376
	 */
	 	printf("init1\r\n");
		exist = usb_check_exist();
		if (exist == 0xFF || exist== 0x00 )
		{	
			printf("init fail: %d\r\n",exist);			
			tft_prints(0,9,"Fail Connect");		
			tft_update();
		}
		else
		break;
	}
	printf("init2\r\n");
	while(1){
	/* config xbox controller via ch376 */	
		if (usb_main_loop() == USB_STATE_CONFIGURED) {
			break;		
    } else {
      check_usb_connection();
    }
	}
	tft_prints(0, 0, "init xbc..OK!");
	tft_update();
}

