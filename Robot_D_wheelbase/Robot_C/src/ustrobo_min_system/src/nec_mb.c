#include "nec_mb.h"
#include "ticks.h"

static NEC_Msg nec_msg[NEC_DEVICE_COUNT];
static u32 last_nec_rx_ticks = 0;

static const NEC_Msg NEC_NULL = {0, 0};

static void nec_mb_decoding(CanRxMsg msg)
{
	if (msg.StdId >= NEC_CAN_ID && msg.StdId < NEC_CAN_ID + NEC_DEVICE_COUNT) {
		if (msg.DLC == 3) {
			if (msg.Data[0] < NEC_DEVICE_COUNT) {
				u8 id = msg.Data[0];
				nec_msg[id].address = (u8) msg.Data[1];
				nec_msg[id].command = (u8) msg.Data[2];
				last_nec_rx_ticks = get_full_ticks();
			}
		}
	}
}

/**
	* @brief NEC (in mainboard) initialization
	*/
void nec_mb_init(void)
{
	can_rx_add_filter(NEC_CAN_ID, CAN_RX_MASK_DIGIT_0_F, nec_mb_decoding);
	for (u8 i = 0; i < NEC_DEVICE_COUNT; ++i) {
		nec_msg[i].address = 0;
		nec_msg[i].command = 0;
	}
}

/**
	* @brief Get the current NEC message
	* @param id: ID of the NEC on the sensor board
	* @retval A const pointer of the NEC message
	*/
const NEC_Msg* nec_get_msg(u8 id)
{
	if (get_full_ticks() - last_nec_rx_ticks > NEC_CAN_RX_TIMEOUT) {
		nec_msg[id] = NEC_NULL; 
	}
	return &nec_msg[id];
}

