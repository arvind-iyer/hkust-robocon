#include "xbc.h"

static u8 state = 0;
static u8 phase = CH376_TKN_PHASE_DATA1;
static u8 xbc_data[XBC_DATA_COUNT] = {0};

u8 get_xbc_data(u8 i) 
{
    if (i >= XBC_DATA_COUNT) {return 0;}
    else {return xbc_data[i];}
}

void xbc_tx_data(void)
{
  if (usb_connected()) {
    CAN_MESSAGE msg[XBC_DATA_COUNT / 8 + (XBC_DATA_COUNT % 8 > 0)] = {{.length = 0}};

    msg[0].length = 0; 
    
    for (u8 i = 0, id = 0; i < XBC_DATA_COUNT; ++i) {
      if (i == 8) {
        ++id;
        msg[id].length = 0;
      }
      msg[id].data[i % 8] = xbc_data[i];
      msg[id].length++;
      msg[id].id = CAN_XBC_BASE + id;
    }

    for (u8 i = 0; i < sizeof(msg) / sizeof(CAN_MESSAGE); ++i) {
      can_tx_enqueue(msg[i]);
    }
  } else {
    // Disconnected
    CAN_MESSAGE msg;
    msg.id = 0x90;
    msg.length = 0;
    can_tx_enqueue(msg); 
  }
  
    /*
  CAN_MESSAGE msg[2];
  msg[0].length = 8;
  msg[0].id = XBC_CAN_ID_BASE + 0;
  for (u8 i = 0; i < 8; ++i) {
    msg[0].data[i] = xbc_data[i];
  }
 
  msg[1].length = 5;
  msg[1].id = XBC_CAN_ID_BASE + 1;  
  for (u8 i = 0; i < 5; ++i) {
    msg[1].data[i] = xbc_data[i + 8];
  }
 
  */

  
}

void xbc_loop(void)
{
  if (!usb_connected()) {
    state = 0;
    for (u8 i = 0; i < sizeof(xbc_data) / sizeof(u8); ++i) {
      xbc_data[i] = 0;
    }
    return;
  }
  
  switch (state) {
    case 0: 
      CHECK_USB_BUFF(CH376_TKN_PHASE_DATA1);
      state = 1; 
    break;
    
    case 1:
    { 
      u8 status = usb_get_status(); 
      
      if (status != CH376_USB_INT_SUCCESS)
      {/* if error occur, leave current process, wait for next process */
        if (status == 0x2B){
          phase &= 0x00;
        }
        else if (status == 0x23){
          phase |= 0x80;
        }
        CHECK_USB_BUFF(phase);
      } else {
        // SUCCESSFUL CONNECTION
        //read_usb_buff();
        ch376_write_cmd(CH376_CMD_RD_USB_DATA0);
        u8 xbc_data_raw[XBC_DATA_RAW_COUNT] = {0};
        
        for (u8 i = 0; i < XBC_DATA_RAW_COUNT; ++i) {
          xbc_data_raw[i] = ch376_read_data();
        }
        
        if (xbc_data_raw[0] == 0x14 && xbc_data_raw[1] == 0x00 && xbc_data_raw[2] == 0x14) {
          // Valid data
          for (u8 i = 0; i < XBC_DATA_COUNT; ++i) {
            xbc_data[i] = xbc_data_raw[i+3]; 
          }
        }
        
        phase ^= USB_PHASING_MASK;
				CHECK_USB_BUFF(phase);
        
      }
    }
  }
}
