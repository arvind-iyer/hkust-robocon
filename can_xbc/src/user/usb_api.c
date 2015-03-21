#include "usb_api.h"

#define TARGET_ADDRESS 0x00 
#define CONFIGURE_VALUE 0x00

u8 _state = USB_STATE_NOT_INIT;
u8 is_usb_ready = 1;
u8 _phasing = CH376_TKN_PHASE_DATA1;
u8 _isRunning = 0;
u8 tkn_x_status = 0;
u8 old_tkn_x_status = 0;
u8 old_pid = 0;
u8 tmp = 0;
u16 usb_conf_cnt = 100;
u16 check_err_exist = 0;
u8 xbc_command_led1[] = {0x03,0x01,0x03,0x02}; //06: first led; 07: second led
u8 xbc_command_led2[] = {0x03,0x01,0x03,0x03}; //06: first led; 07: second led
u8 xbc_command_led3[] = {0x03,0x01,0x03,0x04}; //06: first led; 07: second led
u8 xbc_command_led4[] = {0x03,0x01,0x03,0x05}; //06: first led; 07: second led
u8 xbc_command_rumble[] = {0x06, 0x00, 0x06, 0x00, 0xff, 0x00, 0xff};
u8 xbc_command_rumble2[] = {0x08,0x00,0x08,0x00,0xff,0xff,0x00,0x00,0x00}; //
u8 set_xbc_flag = 1;
u8 get_state_xbc= 0;
u8 array_s = 8;
u8 xbc_id = 3;
u8 usb_get_state(void)
{
  return _state;
}

u8 usb_connected(void)
{
  return _state == USB_STATE_CONFIGURED;
}

void tracking_xbc(void){

	CHECK_USB_BUFF(_phasing);
	tkn_x_status = usb_get_status();
	if (tkn_x_status == CH376_USB_INT_SUCCESS)
	{
		_state = USB_STATE_CONFIGURED;
		_phasing ^= USB_PHASING_MASK;
	}
}


u8 check_ch376_usb_conn(void){
  u8 tmp_status = 0;
  u8 is_connect = 0;	
	
	if (usb_main_loop() != USB_STATE_CONFIGURED)
		while(usb_main_loop() != USB_STATE_CONFIGURED);
	return 1;
  

}

u8 check_usb_data_buff(void){
/*  functoin:
*	1. make sure controller is configured, otherwise means controller disconnected
*	2. change the state to read controller when it ready, otherwise reset again
*/
u8 i = 0;
u8 is_connect = 0;

//	//printf("C3: %d\r\n",tmp);
	if (_state == USB_STATE_CONFIGURED)
	{	
		is_connect = ch376_test_connect();
		if ( is_connect == CH376_USB_INT_USB_READY || is_connect == CH376_USB_INT_CONNECT)
			_state = USB_STATE_TKN_X;
		else if (is_connect == CH376_USB_INT_DISCONNECT){
			////printf("disc 31\r\n");
			_state = USB_STATE_NOT_INIT;	
		}
	}
		
	while(tmp++ < 5 && usb_main_loop() != USB_STATE_CONFIGURED);
		tmp = 0;
		return _state;
	
}

void usb_init(void)
{	
	ch376_hardware_init();
}

u8 get_xbc_id(void)
{
  return xbc_id;
}

void reset_xbc(u8 i) 
{
  ch376_write_cmd(CH376_CMD_RESET_ALL);
  usb_stop_run();
  usb_start_run();
  xbc_id = i;
  
}

/*
 * configure xbox controller and ch376(IC)
 */
u8 usb_main_loop()
{	
////printf("%dt:%d\r\n", tmp,get_ticks());				
	if (!_isRunning)
	{
		return _state;
	}
		
	switch(_state)
	{
	case USB_STATE_NOT_INIT :	
		is_usb_ready = 0;
		////tft_clear();
		////tft_prints(0, 2, "not init...");
		//printf("not init %d\r\n",usb_conf_cnt);			
		if (usb_conf_cnt++ > 150){ // usb_conf_cnt = 100
			usb_conf_cnt = 100;
			_state = USB_STATE_DISCONNECTING;
		}
		break;
		
	case USB_STATE_DISCONNECTING :		
		
		//printf("disconnecting\r\n");	
		
		if (usb_disconnect() == CH376_CTRL_STATUS_SUCCESS){		
			if (ch376_test_connect() == CH376_USB_INT_CONNECT)
			{			
				_state = USB_STATE_ATTACHED;
			}
			else if (ch376_test_connect() == CH376_USB_INT_DISCONNECT){
				/* print disconnect msg */
        //tft_clear();
				//tft_prints(0,0,"USB DISCONNECT!!!");
				//tft_update();
				_state = USB_STATE_DISCONNECTED;
			}
		}
		
		break;
		
	case USB_STATE_DISCONNECTED :
		//printf("disconnected\r\n");
			if (usb_test_connection() == CH376_USB_INT_CONNECT)
			{			
				if (usb_disconnect() == CH376_CTRL_STATUS_SUCCESS){ //05h
					_state = USB_STATE_ATTACHED;	
				}
				else{
					if (usb_get_status() == CH376_USB_INT_USB_READY)
						_state = USB_STATE_RESETED;
				}
					
			}
			else if (ch376_test_connect() == CH376_USB_INT_DISCONNECT){
				//printf("1\r\n");
				_state = USB_STATE_NOT_INIT;		
			}
	
		break;
		
	case USB_STATE_ATTACHED :
	//	//tft_prints(0, 5, "attached.%d", get_seconds() % 10);
		//printf("attached\r\n");
		{
			if (usb_reset() == CH376_CTRL_STATUS_SUCCESS) //07h
			{			
				_state = USB_STATE_RESETED;
			}
			else if (usb_test_connection() == CH376_USB_INT_DISCONNECT)
			{
				//printf("12\r\n");
				_state = USB_STATE_NOT_INIT;
			}
			else{
				if (usb_get_status() == CH376_USB_INT_USB_READY)
					_state = USB_STATE_RESETED;
			}
								
		}
		break;		
		
	case USB_STATE_RESETED :
		//tft_prints(0, 4, "%x.reset..%d",ch376_test_connect(), get_seconds() % 10);
		//printf("reseted\r\n");	
		if (usb_connect() == CH376_CTRL_STATUS_SUCCESS) //06h
		{
			usb_set_speed(USB_12M_FULL_SPEED);
			_state = USB_STATE_SETSPEED;					
		}	
		else if (usb_test_connection() == CH376_USB_INT_DISCONNECT){
			_state = USB_STATE_NOT_INIT;
			//printf("13\r\n");
			}
		break;
	
	case USB_STATE_SETSPEED :
		
		//tft_prints(0, 5, "auto set..%xh",usb_get_status());
		//printf("autoset\r\n");
		
		if (usb_get_status() == CH376_USB_INT_SUCCESS || --usb_conf_cnt < 80 ) // usb_conf_cnt = 100
			{			
			//	//printf("autoset by cnt\r\n");
				_state = USB_STATE_AUTOSET;
				usb_conf_cnt = 100;
		
			}
			else if (usb_test_connection() == CH376_USB_INT_DISCONNECT)
			{
			//	//printf("autoset disconn\r\n");
				//printf("14\r\n");
				_state = USB_STATE_NOT_INIT;
			}
			else
				usb_set_speed(USB_12M_FULL_SPEED);			
		
		break; 

	case USB_STATE_AUTOSET :
 	////tft_prints(0, 5, "check config..");
	//	//printf("check config\r\n");
			usb_auto_setup();
			if (usb_get_status() == CH376_USB_INT_SUCCESS)
			{
				_state = USB_STATE_CONFIGURED;	
				//xbc_config(1);
        xbc_config(xbc_id); 
        
			}
			else if (usb_test_connection() == CH376_USB_INT_DISCONNECT)
			{
				////printf("16\r\n");
				_state = USB_STATE_NOT_INIT;
			}		
	
		break;
	
	case USB_STATE_CONFIGURED :
		is_usb_ready = 1;
		if ((get_ticks() % UPDATE_TIME) == 0) //every 20ms
		{		
			if (usb_test_connection() == CH376_USB_INT_DISCONNECT)
			{	
			//	//printf("17\r\n");
				_state = USB_STATE_NOT_INIT;
			}
		}
		break;
		
	case USB_STATE_TKN_X :
			usb_conf_cnt = 250;
			is_usb_ready = 1;
			
			CHECK_USB_BUFF(_phasing);		
			
			while(HV_INT){
		
			};
			tkn_x_status = usb_get_status();
			
			while (1) {
				if (tkn_x_status != 0x00 && tkn_x_status != 0x18 ) break;
				tkn_x_status = usb_get_status();			
			}
			
			if (tkn_x_status == CH376_USB_INT_SUCCESS)
			{
				_state = USB_STATE_CONFIGURED;
				_phasing ^= USB_PHASING_MASK;
			}
			else if (tkn_x_status == 0x2B) {
				_phasing &= 0x00;				
			}				
			else if (tkn_x_status == 0x23) {		
				_phasing |= 0x80;			
			}
			else if (usb_test_connection() == CH376_USB_INT_DISCONNECT )
			{
				//printf("19\r\n");
				_state = USB_STATE_NOT_INIT;
			}
		
		break;
	case USB_STATE_SUSPENDED :
		break;
	case USB_STATE_ERROR :
		
		break;
	default:
		//_state = USB_STATE_ERROR;
		break;
	}
	//tft_update();
	return _state;
}

void xbc_config (u8 function) 
{
  u8 i =0;
			//_phasing ^= USB_PHASING_MASK;
			ch376_write_cmd(CH376_CMD_WR_HOST_DATA);	//write data to host		

			if (function == 1){
				for(i=0; i < xbc_command_led1[0]+1;i++){		
					ch376_write_data(xbc_command_led1[i]);					
				}
			}
			
			if (function == 2){
				for(i=0; i < xbc_command_led2[0]+1;i++){		
					ch376_write_data(xbc_command_led2[i]);					
				}
			}
			
			if (function == 3){
				for(i=0; i < xbc_command_led3[0]+1;i++){		
					ch376_write_data(xbc_command_led3[i]);					
				}
			}
			
			if (function == 4){
				for(i=0; i < xbc_command_led4[0]+1;i++){		
					ch376_write_data(xbc_command_led4[i]);					
				}
			}
			else if (function == 5){
				for(i=0; i <  array_s ;i++){		
				ch376_write_data(xbc_command_rumble[i]);					
			}
			}
			else if (function == 6){
				for(i=0; i < 9;i++){		
				ch376_write_data(xbc_command_rumble2[i]);					
			}
			}			
			if (function == 5 || function == 6){
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			_delay_ms(1000);
			
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			_delay_ms(1000);
			}
			
			else if (function == 8)
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			
			else
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x11);  // send data to usb
		
}

u8 set_xbc_config(u8 function){

//u8 xbc_command[] = {0x03,0x01,0x03,0x06};
	u8 i = 0;
	switch(set_xbc_flag){
		
		case 1:
			_phasing ^= USB_PHASING_MASK;
			ch376_write_cmd(CH376_CMD_WR_HOST_DATA);	//write data to host	
			set_xbc_flag++;
			
		break;
		
		case 2:
		
			if (function == 1){
				for(i=0; i < xbc_command_led1[0]+1;i++){		
					ch376_write_data(xbc_command_led1[i]);					
				}
			}
			
			if (function == 2){
				for(i=0; i < xbc_command_led2[0]+1;i++){		
					ch376_write_data(xbc_command_led2[i]);					
				}
			}
			
			if (function == 3){
				for(i=0; i < xbc_command_led3[0]+1;i++){		
					ch376_write_data(xbc_command_led3[i]);					
				}
			}
			
			if (function == 4){
				for(i=0; i < xbc_command_led4[0]+1;i++){		
					ch376_write_data(xbc_command_led4[i]);					
				}
			}
			else if (function == 5){
				for(i=0; i <  array_s ;i++){		
				ch376_write_data(xbc_command_rumble[i]);					
			}
			}
			else if (function == 6){
				for(i=0; i < 9;i++){		
				ch376_write_data(xbc_command_rumble2[i]);					
			}
			}
	
			set_xbc_flag++;
			
		break;
		
		case 3:
			
			if (function == 5 || function == 6){
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			_delay_ms(1000);
			
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			_delay_ms(1000);
			}
			
			else if (function == 8)
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x19);  // send data to usb
			
			else
			ch376_write_cmd_data2(CH376_CMD_ISSUE_TKN_X, 0x00, 0x11);  // send data to usb
		
			
			set_xbc_flag++;
			
		break;
		
		case 4:

			if (1){
				set_xbc_flag++;
				//tft_prints(0,2, "set...OK!:%x",get_state_xbc);
			}
			else
				set_xbc_flag = 1;
				
			//tft_prints(0,1, "setting..:%x",get_state_xbc);
		
		break;
		
		case 5:// finish set the xbc
			set_xbc_flag = 1;
			return 1;
			//do nothing
		break;
		
		default:
			set_xbc_flag = 1;
		break;
	
	}
//	//tft_prints(15, 9, "%d", get_seconds() % 10);
//	//tft_update();
	return 0;

}


void usb_start_run(void)
{
	if (_isRunning)
	{
		return;
	}
	
	_isRunning = 1;
	//printf("f1\r\n");
	_state = USB_STATE_NOT_INIT;
}

void usb_stop_run(void)
{
	if (!_isRunning)
	{
		return;
	}
	
	_isRunning = 0;
	//printf("s1\r\n");
	_state = USB_STATE_NOT_INIT;
}

u8   get_usb_state(void)
{
	return _state;
}

void usb_set_speed(u8 speed)
{
	ch376_set_usb_speed(speed);
}

void usb_set_retry_times(u8 times)
{
	ch376_set_retry(times);
}

u8   usb_disconnect(void)
{
	return ch376_set_usb_mode(CH376_MODE_HOST_NON_SOF);
	
}

u8   usb_reset(void)
{
	return ch376_set_usb_mode(CH376_MODE_HOST_RST);
	
}

u8   usb_connect(void)
{
	return ch376_set_usb_mode(CH376_MODE_HOST_SOF);
	
}

u8   usb_test_connection(void)
{
	return ch376_test_connect();
}

u8   usb_hv_interrupt(void)
{
	return HV_INTERRUPT();
}

u8   usb_get_status(void)
{	
	return ch376_get_int_status();
}

u8   usb_check_exist(void)
{
	return ch376_check_exist(CHECK_EXIST_DATA);
}

void usb_auto_setup(void){
	ch376_write_cmd(CH376_CMD_AUTO_SETUP);
	_delay_ms(20);
}


u8 read_tkn_status(void){
	return tkn_x_status;
}
