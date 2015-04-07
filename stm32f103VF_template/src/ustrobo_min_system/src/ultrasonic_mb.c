#include "ultrasonic_mb.h"

static US_MB_Typedef us_mb_devices[US_DEVICE_COUNT] = {
	{0}
};


static void us_mb_decoding(CanRxMsg msg)
{
	if (msg.StdId >= US_CAN_ID && msg.StdId < US_CAN_ID + US_DEVICE_COUNT) {
		if (msg.DLC == 2) {
			u8 id = msg.StdId - US_CAN_ID;
			us_mb_devices[id].distance = (msg.Data[0] << 8) | msg.Data[1];
		}
	}
}


void us_mb_init(void)
{
	can_rx_add_filter(US_CAN_ID, CAN_RX_MASK_DIGIT_0_F, us_mb_decoding);
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		us_mb_devices[i].distance = 0;
	}
}


u16 us_get_distance(u8 id)
{
	if (id >= US_DEVICE_COUNT) {return 0;}
	return us_mb_devices[id].distance;
}

