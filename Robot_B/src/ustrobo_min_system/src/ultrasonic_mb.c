#include "ultrasonic_mb.h"
#include "ticks.h"

static US_MB_Typedef us_mb_devices[US_DEVICE_COUNT] = {
	{0}
};
static u32 us_last_can_rx = 0;


static void us_mb_decoding(CanRxMsg msg)
{
	if (msg.StdId >= US_CAN_ID && msg.StdId < US_CAN_ID + US_DEVICE_COUNT) {
		if (msg.DLC == US_CAN_DISTANCE_LENGTH && msg.Data[0] == US_CAN_DISTANCE_CMD) {
			u8 id = msg.StdId - US_CAN_ID;
			us_mb_devices[id].distance = (msg.Data[1] << 8) | msg.Data[2];
			us_last_can_rx = get_full_ticks();
		}
		
		if (msg.DLC == US_CAN_PROC_LENGTH && msg.Data[0] == US_CAN_PROC_CMD) {
			u8 id = msg.StdId - US_CAN_ID;
			u16 in_range_time = (msg.Data[1] << 8) | msg.Data[2];
			u16 in_range_distance = (msg.Data[3] << 8) | msg.Data[4];
			// TODO:
			
		}
	}
}


/**
	* @brief Ultrasonic initialization (on mainboard)
	*/
void us_mb_init(void)
{
	can_rx_add_filter(US_CAN_ID, CAN_RX_MASK_DIGIT_0_F, us_mb_decoding);
	for (u8 i = 0; i < US_DEVICE_COUNT; ++i) {
		us_mb_devices[i].distance = 0;
	}
}


/**
	* @brief Get the detected distance of the ultrasonic sensor
	* @param id: ID of the ultrasonic sensor (0 to US_DEVICE_COUNT)
	* @retval The detected distance of the ultrasonic sensor in millimeter (mm)
	*/
u16 us_get_distance(u8 id)
{
	if (id >= US_DEVICE_COUNT) {return 0;}
	if (get_full_ticks() - us_last_can_rx > US_CAN_RX_TIMEOUT) {return 0;}
	return us_mb_devices[id].distance; 
}

