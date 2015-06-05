#include "ks103.h"
#include "ticks.h"
#include "delay.h"

static bool ks103_init_flag = false;

void ks103_rx(u8 id, u8 data);

static void KS103_decode0(u8 data) 
{
	ks103_rx(0, data); 
}

static void KS103_decode1(u8 data) 
{
	ks103_rx(1, data); 
}


static KS103_TypeDef devices[] = {
	{
		.COMx = COM1,
		.range_command = 0xb0,
		.rx_handler = KS103_decode0
	},
	{
		.COMx = COM3,
		.range_command = 0xb0,
		.rx_handler = KS103_decode1
	}
};

void ks103_rx(u8 id, u8 data) {
	if (id >= KS103_COUNT || !ks103_init_flag) {return;}

	KS103_TypeDef* device = &devices[id];
	if (device->state == KS103_SEND_DETECTING_COMMAND) {
		device->raw_data_high = data;
		device->state = KS103_RECEIVE_HIGH_BIT;
	} else if (device->state == KS103_RECEIVE_HIGH_BIT) {
		device->raw_data_low = data;
		device->state = KS103_NULL;
		device->range = ((device->raw_data_high & 0xFF) << 8) | (device->raw_data_low & 0xFF);
		
		device->sample_count_tmp++;
		
		if (device->last_sample_second != get_seconds()) {
			device->last_sample_second = get_seconds();
			device->sample_count = device->sample_count_tmp;
			device->sample_count_tmp = 0;	// Reset
		}
		
		//ks103_tx(id);
	}
}

void ks103_init(void)
{
	ks103_init_flag = false;
	for (u8 i = 0; i < KS103_COUNT; ++i) {
		KS103_TypeDef* device = &devices[i];
		uart_init(device->COMx, KS103_BR);
		uart_rx_init(device->COMx, device->rx_handler);
		device->raw_data_low = 0;
		device->raw_data_high = 0;
		device->state = KS103_NULL;
		device->range = 0;
		
		device->last_sample_second = 0;
		device->sample_count_tmp = 0;
		device->sample_count = 0;
		
		uart_tx_byte(device->COMx, KS103_TTL_ADDRESS_DEFAULT);
		_delay_us(50);
		uart_tx_byte(device->COMx, KS103_TTL_REGISTER_DEFAULT);
		_delay_us(50);
		uart_tx_byte(device->COMx, 0x72);	// Noise reduction command

		uart_tx_byte(device->COMx, KS103_TTL_ADDRESS_DEFAULT);
		_delay_us(50);
		uart_tx_byte(device->COMx, KS103_TTL_REGISTER_DEFAULT);
		_delay_us(50);
		uart_tx_byte(device->COMx, 0x0f);	// Range command
	}
	ks103_init_flag = true;
	
}
bool ks103_tx(u8 id)
{
	KS103_TypeDef* device = &devices[id];
	if (device->state != KS103_NULL || id >= KS103_COUNT) {
		return false; 
	} else {
		uart_tx_byte(device->COMx, KS103_TTL_ADDRESS_DEFAULT);
		//_delay_us(50);
		uart_tx_byte(device->COMx, KS103_TTL_REGISTER_DEFAULT);
		//_delay_us(50);
		uart_tx_byte(device->COMx, device->range_command);
		device->state = KS103_SEND_DETECTING_COMMAND;  
	}
}

void ks103_tx_force(u8 id)
{
	KS103_TypeDef* device = &devices[id];
	uart_tx_byte(device->COMx, KS103_TTL_ADDRESS_DEFAULT);
	//_delay_us(50);
	uart_tx_byte(device->COMx, KS103_TTL_REGISTER_DEFAULT);
	//_delay_us(50);
	uart_tx_byte(device->COMx, device->range_command);
	device->state = KS103_SEND_DETECTING_COMMAND;  
}


const KS103_TypeDef* ks103_get(u8 id) 
{
	if (id >= KS103_COUNT) {return 0;}
	return &devices[id];
}

