#include "system_test.h"

static u16 ticks_img 	= (u16)-1;
static u8 received_data[5] = {0};

static u8 return_listener(void)
{
	return button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10;
}

void battery_test(void)
{
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BATTERY TEST");
				tft_prints(0, 2, "ADC: %d", get_battery_adc());
				tft_prints(0, 3, "V: %d", get_voltage());
				tft_prints(0, 4, "V avg: %d", get_voltage_avg());
				tft_update();
			}
		}
	}
}

void bluetooth_test(void)
{
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "Received: %d", bluetooth_get_data_count());
				tft_prints(0, 2, "Recent data: ");
				tft_prints(0, 3, "ID: 0x%X", bluetooth_recent_rx_id());
				tft_prints(0, 4, "Length: %d", bluetooth_recent_rx_data_length());
				tft_prints(0, 5, "Data: ");
				const u8* data = bluetooth_recent_rx_data();
				tft_prints(0, 6, "{%02X,%02X,%02X,%02X,", data[0], data[1], data[2], data[3]);
				tft_prints(0, 7, " %02X,%02X,%02X,%02X}", data[4], data[5], data[6], data[7]);
				tft_update();
			}
		}
	}		
}

void ascii_test(void)
{
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				
				static u8 char_start = STARTING_ASCII;
				
				if (button_pressed(BUTTON_JS2_DOWN) == 1 || button_pressed(BUTTON_JS2_UP) == 1) {
					char_start = (char_start == STARTING_ASCII) ? STARTING_ASCII + tft_get_max_x_char() * (tft_get_max_y_char() - 2) : STARTING_ASCII;
					CLICK_MUSIC;
				}
				char char_i = char_start; 
				/** Display characters from ascii value 0 **/
				for (u8 y = 2; y < tft_get_max_y_char(); ++y) {
					for (u8 x = 0; x < tft_get_max_x_char(); ++x) {
						tft_prints(x, y, "%c", char_i);
						++char_i;
					}
				}	
				
				tft_prints(0, 1, "ASCII (%d-%d)", char_start, (u8) (char_i - 1));
				tft_update();
			}
		}
	}	
}

void motor_test(void)
{	
	TFT_UI_ITEM
		motor_list = {
			.type = tft_ui_list,
			.x = 2, .y = 2,
			.ui_item.list = {
				.width = 10,
				.range.lower = 0,
				.range.upper = CAN_MOTOR_COUNT-1,
				.selected_int = 0
			}	
		},
		
		motor_loop_type = {
			.type = tft_ui_list,
			.x = 2, .y = 5,
			.ui_item.list = {
				.width = 10,
				.range.lower = (s32) OPEN_LOOP,
				.range.upper = (s32) CLOSE_LOOP,
				.selected_int = 0
			}
		},
		
		motor_speed = {
			.type = tft_ui_list,
			.x = 7, .y = 6,
			.ui_item.list = {
				.width = 6,
				.range.lower = (s32) -999,
				.range.upper = (s32) 999,
				.selected_int = 0
			}
		},
		
		motor_test_flag = {
			.type = tft_ui_checkbox,
			.x = 1, .y = 7,
			.ui_item.checkbox = {
				.checked = false
			}
		};  
	
	TFT_UI_ITEM* motor_test_ui_list[] = {
		&motor_list, &motor_loop_type, &motor_speed, &motor_test_flag
	}; 
	
	TFT_UI tft_ui = {
		4, motor_test_ui_list, 0
	};
	
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					// Stop the motor before exit the test.
					motor_set_vel((MOTOR_ID) tft_ui_get_val(&motor_list), 0, OPEN_LOOP);
					return; 
				}
				// Not allow to move in test mode
				if (!tft_ui_get_val(&motor_test_flag)) {
					// Line switcher
					if (button_pressed(BUTTON_JS2_UP) == 1) {				
						tft_ui_listener(&tft_ui, tft_ui_event_up);
					}
					
					if (button_pressed(BUTTON_JS2_DOWN) == 1) {
						tft_ui_listener(&tft_ui, tft_ui_event_down);
					}
				}
				
				if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
					tft_ui_listener(&tft_ui, tft_ui_event_left);
				}
				
				if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
					tft_ui_listener(&tft_ui, tft_ui_event_right);
				}
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					tft_ui_listener(&tft_ui, tft_ui_event_select);
				}

			}
			
			if (ticks_img % 50 == 5) {
				if (tft_ui_get_val(&motor_test_flag)) {
					// Start setting velocity
					motor_set_vel((MOTOR_ID) tft_ui_get_val(&motor_list), (s32) tft_ui_get_val(&motor_speed), \
						(CLOSE_LOOP_FLAG) tft_ui_get_val(&motor_loop_type));
				} else {
					// Stop
					motor_set_vel((MOTOR_ID) tft_ui_get_val(&motor_list), 0, OPEN_LOOP);
				}
			}
			
			if (ticks_img % 50 == 7) {
				tft_clear();
				draw_top_bar();
				
				tft_prints(0, 1, "MOTOR TEST");
				tft_prints(5, 2, "MOTOR%-2d", tft_ui_get_val(&motor_list) + 1);
				tft_prints(0, 3, "ID: 0x%03X", CAN_MOTOR_BASE + tft_ui_get_val(&motor_list)); 
				tft_prints(0, 4, "Encoder: %d", get_encoder_value((MOTOR_ID) tft_ui_get_val(&motor_list)));				
				tft_prints(6, 5, "%s", tft_ui_get_val(&motor_loop_type) == CLOSE_LOOP ? "CLOSE" : "OPEN");
				tft_prints(0, 6, "Speed:   %d", tft_ui_get_val(&motor_speed));
				tft_prints(0, 7, "   Start test");
				
				tft_ui_update(&tft_ui, ticks_img % 500 < 250);
				tft_update();
			}
		}
	}		
}

void position_test(void)
{
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					gyro_cal();
					CLICK_MUSIC;
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "POSITION TEST");
        tft_prints(0, 2, "  Pos(Raw)");
				tft_prints(0, 3, " X:%4d(%4d)", get_pos()->x, get_pos_raw()->x);
				tft_prints(0, 4, " Y:%4d(%4d)", get_pos()->y, get_pos_raw()->y);
				tft_prints(0, 5, " A:%4d(%4d)", get_pos()->angle, get_pos_raw()->angle);
				tft_prints(0, 6, " Avail: %d", gyro_available);
				tft_prints(0, 8, " (%c) Calibrate", ticks_img < 500 ? BLACK_BLOCK_ASCII : ' ');
				
				tft_update();
			}
		}
	}
}

void button_test(void)
{
	while (1) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BUTTON TEST");
				for (u8 i = 0, x = 0, y = 2; i < BUTTON_COUNT; ++i) {
					tft_prints(x, y, "%d:%d(%d)", i, button_pressed((BUTTON) i), button_released((BUTTON) i));
					if (y < tft_get_max_y_char() - 1) {
						++y;
					} else {
						x += 7;
						y = 2;
					}
				}
				tft_update();
			}
		}
	}	
}

static void buzzer_onclick_listener(TFT_UI_ITEM** buzzer_item)
{
  MUSIC_NOTE curr_note[] = {{(MUSIC_NOTE_LETTER) tft_ui_get_val(buzzer_item[0]), tft_ui_get_val(buzzer_item[1])}, {NOTE_END, 0}};
  buzzer_stop_song();									                            	// stop last note
  buzzer_set_volume(tft_ui_get_val(buzzer_item[2]));					    	// set with given volume
  buzzer_play_song(curr_note, tft_ui_get_val(buzzer_item[3]), 0);	  // Play the note.	
}

void buzzer_test(void)
{
  TFT_UI_ITEM
    note_pitch = {
      .type = tft_ui_list,
      .x = 9, .y = 2,
      .ui_item.list = {
        .width = 5,
        .range.lower = (s32) NOTE_REST,
        .range.upper = (s32) NOTE_B,
        .selected_int = (s32) NOTE_C
      }
    },
    
    note_octave = {
      .type = tft_ui_list,
      .x = 9, .y = 3, 
      .ui_item.list = {
        .width = 5,
        .range.lower = 1,
        .range.upper = 10,
        .selected_int = 6
      }
    },
    
    note_volume = {
      .type = tft_ui_list,
      .x = 9, .y = 4,
      .ui_item.list = {
        .width = 5,
        .range.lower = 1,
        .range.upper = 100,
        .selected_int = 30
      }
    },
    
    note_duration = {
      .type = tft_ui_list,
      .x = 9, .y = 5,
      .ui_item.list = {
        .width = 5,
        .range.lower = 10,
        .range.upper = 65535,
        .selected_int = 100
      }
    };
    
  TFT_UI_ITEM* buzzer_test_ui_list[] = {
    &note_pitch, &note_octave, &note_volume, &note_duration
  };
  
  TFT_UI tft_ui = {
		4, buzzer_test_ui_list, 0, buzzer_onclick_listener
	};
  char rest = (char) 22;
	const char* note_char[] = {&rest, "C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
  
	while (true) {
		if (ticks_img != get_ticks()) {			
			ticks_img = get_ticks();			
						
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					buzzer_set_volume(30); 	// Return to default volume
					buzzer_stop_song();	
					return; 
				}
				
				// Line switcher
				if (button_pressed(BUTTON_JS2_UP) == 1 || button_hold(BUTTON_JS2_UP, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_up);
				}
				
				if (button_pressed(BUTTON_JS2_DOWN) == 1 || button_hold(BUTTON_JS2_DOWN, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_down);
				}

				if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_left);
          if (tft_ui_get_val(&note_pitch) == NOTE_REST) {
            note_pitch.ui_item.list.selected_int = NOTE_B;
            if (tft_ui_get_val(&note_octave) > note_octave.ui_item.list.range.lower) {
              --note_octave.ui_item.list.selected_int;
            } else {
              note_octave.ui_item.list.selected_int = note_octave.ui_item.list.range.upper;
            }
          }
				}
				
				if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_right);
          if (tft_ui_get_val(&note_pitch) == NOTE_REST) {
            note_pitch.ui_item.list.selected_int = NOTE_C;
            if (tft_ui_get_val(&note_octave) < note_octave.ui_item.list.range.upper) {
              ++note_octave.ui_item.list.selected_int;
            } else {
              note_octave.ui_item.list.selected_int = note_octave.ui_item.list.range.lower;
            }
          }
        }
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					tft_ui_listener(&tft_ui, tft_ui_event_select);
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BUZZER TEST");
				tft_prints(0, 2, "Notes: ");
        tft_prints(11, 2, "%s", note_char[tft_ui_get_val(&note_pitch)]);
				tft_prints(0, 3, "Octave");
				tft_prints(11, 3, "%d", tft_ui_get_val(&note_octave));
				tft_prints(0, 4, "Volume: ");
        tft_prints(11, 4, "%d", tft_ui_get_val(&note_volume));
				tft_prints(0, 5, "Duration:");
        tft_prints(11, 5, "%d", tft_ui_get_val(&note_duration));
				if (ticks_img < 500) {
					tft_prints(1, 7, "Click to play");
				}
        tft_ui_update(&tft_ui, ticks_img % 500 < 250);
				tft_update();
			}
		}
	}
}

static void can_on_click_listener(TFT_UI_ITEM** can_item)
{
  CAN_MESSAGE txmsg;
  txmsg.id = tft_ui_get_val(can_item[0]);
  txmsg.length = tft_ui_get_val(can_item[1]);
  for (u8 i = 0; i < txmsg.length; ++i) {
    txmsg.data[i] = 0x00;
  }
  CLICK_MUSIC;
  can_tx_enqueue(txmsg);
}

void can_test(void)
{
  TFT_UI_ITEM
    can_id = {
      .type = tft_ui_list,
      .x = 7, .y = 2,
      .ui_item.list = {
        .width = 7,
				.range.lower = 0x00,
				.range.upper = 0xFF,
				.selected_int = 0
			}
		},  
    
    can_length = {
      .type = tft_ui_list,
      .x = 11, .y = 3,
      .ui_item.list = {
        .width = 3,
				.range.lower = 1,
				.range.upper = 8,
				.selected_int = 1
			}
		};
  
  TFT_UI_ITEM* can_test_ui_list[] = {
		&can_id, &can_length
	};
    
  TFT_UI tft_ui = {
		.item_count = 2, .item_list = can_test_ui_list, .selected_item = 0, .click_event = can_on_click_listener
	};
  
  while (true) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      if (ticks_img % 50 == 0) {
        battery_adc_update();
      }
      if (ticks_img % 50 == 3) {
        button_update();
        if (return_listener()) {
          return;
        }
        // Line switcher
        if (button_pressed(BUTTON_JS2_UP) == 1) {				
          tft_ui_listener(&tft_ui, tft_ui_event_up);
        }
        
        if (button_pressed(BUTTON_JS2_DOWN) == 1) {
          tft_ui_listener(&tft_ui, tft_ui_event_down);
        }
        // Line
        if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_left);
				}
				
				if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_right);
				}
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					tft_ui_listener(&tft_ui, tft_ui_event_select);
				}
      }
      if (ticks_img % 50 == 6) {
        tft_clear();
        draw_top_bar();
        tft_prints(0, 1, "CAN TEST");
        tft_prints(0, 2, "CAN id:");
        tft_prints(9, 2, "00 %02X", tft_ui_get_val(&can_id));
        tft_prints(0, 3, "CAN length:");
        tft_prints(13, 3, "%1d", tft_ui_get_val(&can_length));
        // Only motor related encoder can be receieve, to be improved.
        if (can_get_rx_count() > 0) {
          tft_prints(0, 5, "CAN Rx id: 00 %02X", can_get_recent_rx().id);
          u8 x = 0, y = 6;
          tft_prints(x++, y, "{"); // Caution: MUST BE x++, ++x is wrong!
          for (u8 i = 0; i < can_get_recent_rx().length; ++i) {
            // 2 space for 2 digitt, one space for '}', so minus 3
            if (x > CHAR_MAX_X_VERTICAL - 3) {
              x = 1;
              ++y;
            }
            tft_prints(x, y, "%02X ", can_get_recent_rx().data[i]);
            x += 3;
          }
          tft_prints(x - 1, y, "}"); // No extra space need, minus 1
        } else {
          tft_prints(0, 5, "No CAN received");
        }
        tft_prints(0, 8, "Received no.: %d", can_get_rx_count());
        if (ticks_img < 500) {
          tft_prints(1, 9, "Click to send");
        }
        tft_ui_update(&tft_ui, ticks_img % 500 < 250);
        tft_update();
      }
    }
  }
}

void xbc_test(void)
{
	u8	pressed_cnt = 0;
	u16 press_times = 0;
	u32 pre_xbc_press = 0;
  u32 curr_button = XBC_START;
  char* curr_stage_msg = "";
	while (true) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 50 == 0) {
				battery_adc_update();
			}
			if (ticks_img % 50 == 3) {
				button_update();
				if (return_listener()) {
					return; 
				}
				// Skip or backward
				if (button_pressed(BUTTON_1) == 1 && pressed_cnt > 0) {
					--pressed_cnt;
				}
				if (button_pressed(BUTTON_2) == 1 && pressed_cnt < 19) {
					++pressed_cnt;
				}
			}
			
			if (ticks_img % 50 == 6){
				pre_xbc_press = xbc_press;
				xbc_update();
				tft_clear();
				switch (pressed_cnt) {
					case 0:	
						curr_stage_msg = "PRESS [START]";
						curr_button = XBC_START;
					break;

					case 1:
						curr_stage_msg = "PRESS [UP]";
						curr_button =  XBC_UP;
					break;

					case 2:
						curr_stage_msg = "PRESS [DOWN]   ";
						curr_button =  XBC_DOWN;
					break;

					case 3:
						curr_stage_msg = "PRESS [LEFT]   ";
						curr_button =  XBC_LEFT;
					break;

					case 4:
						curr_stage_msg = "PRESS [RIGHT]   ";
						curr_button =  XBC_RIGHT;
					break;

					case 5:
						curr_stage_msg = "PRESS [BACK]   ";
						curr_button =  XBC_BACK;
					break;

					case 6:
						curr_stage_msg = "PRESS [A]     ";
						curr_button =  XBC_A;
					break;

					case 7:
						curr_stage_msg = "PRESS [B]    ";
						curr_button =  XBC_B;
					break;

					case 8:
						curr_stage_msg = "PRESS [X]    ";
						curr_button =  XBC_X;
					break;

					case 9:
						curr_stage_msg = "PRESS [Y]     ";
						curr_button =  XBC_Y;
					break;

					case 10:
						curr_stage_msg = "PRESS [LB ]   ";
						curr_button =  XBC_LB;
					break;

					case 11:
						curr_stage_msg = "PRESS [RB ]   ";
						curr_button =  XBC_RB;
					break;

					case 12:
						curr_stage_msg = "PRESS [XBOX  ]   ";
						curr_button =  XBC_XBOX;
					break;

					case 13:
						curr_stage_msg = "[L BUTTON 1]";
						curr_button =  L_BUT1;
					break;

					case 14:
						curr_stage_msg = "[L BUTTON 2]";
						curr_button =  L_BUT2;
					break;

					case 15:
						curr_stage_msg = "[L BUTTON 3]";
						curr_button =  L_BUT3;
					break;

					case 16:
						curr_stage_msg = "[R BUTTON 1]";
						curr_button =  R_BUT1;
					break;

					case 17:
						curr_stage_msg = "[R BUTTON 2]";
						curr_button =  R_BUT2;
					break;

					case 18:
						curr_stage_msg = "[R BUTTON 3]";
						curr_button =  R_BUT3;
					break;

					case 19:
						curr_stage_msg = "press [BACK] exit";
						tft_prints(0,3,"KEY TEST OK");
						if (xbc_press & XBC_BACK) {
							return;
						}
          break;
				}

        if (xbc_press & curr_button) {
          ++pressed_cnt;
          SUCCESSFUL_MUSIC;
        }
        
				if (xbc_press != pre_xbc_press && xbc_press != 0) {
					++press_times;
				}
        
				draw_top_bar();
				tft_prints(0,1,"XBOX TEST");
        tft_prints(0,2,curr_stage_msg);
				tft_prints(0,4,"LT:%3ld RT:%3ld",xbc_joy[XBC_LT],xbc_joy[XBC_RT]);
				tft_prints(0,5,"LX:%5ld",xbc_joy[XBC_LX]);
				tft_prints(0,6,"LY:%5ld",xbc_joy[XBC_LY]);
				tft_prints(0,7,"RX:%5ld",xbc_joy[XBC_RX]);
				tft_prints(0,8,"RY:%5ld",xbc_joy[XBC_RY]);
				tft_prints(0,9,"press cnt:%3d",press_times); //normally press once count up 1
				tft_update();
			}
		}
	}
}


/**
	* @brief 	  Pin test for GPIOE only (will have other later)
	* @param 	  None
	* @retval	  None
	* @warning  Reinit is required after this test (except those without GPIO)!!
	*/
void gpio_pin_test(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	/* Pin E test */
	u16 curr_test_pin = GPIO_Pin_0;
	vu16 initial_input = GPIO_ReadInputData(GPIOE);
	u16 last_input = GPIO_Pin_0;		
	u8 pin_no = 0;
	bool test_done = false;
	
	while (!return_listener()) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			if (ticks_img % 50 == 3) {
				button_update();
				if (button_pressed(BUTTON_1) == 1 && pin_no > 0) {
					if (test_done) {
						//return to previous stage.
						test_done = false;		
					} else {
						--pin_no;
						curr_test_pin >>= 1;
					}
				}
				if ((button_pressed(BUTTON_JS2_CENTER) == 1 && test_done) && last_input == initial_input) {
					return;
				}
				
				if (button_pressed(BUTTON_2) == 1 && pin_no < 15) {
					++pin_no;
					curr_test_pin <<= 1;
				}
				
				if (!test_done) {
					if (((GPIO_ReadInputData(GPIOE) ^ initial_input) == curr_test_pin) && last_input == initial_input) {
						SUCCESSFUL_MUSIC;
						if (curr_test_pin != GPIO_Pin_15) {
							curr_test_pin <<= 1;
							++pin_no;
						} else {
							test_done = true;
						}
					} else if (GPIO_ReadInputData(GPIOE) != initial_input && last_input == initial_input) {
						FAIL_MUSIC;
					}
				}
				last_input = GPIO_ReadInputData(GPIOE);
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				if (!test_done) {
					tft_prints(0, 1, "Pin Test");
					tft_prints(0, 2, "Plug to B9");
					tft_prints(0, 3, "Now plug: E%d", pin_no);
					tft_prints(0, 4, "Caution:");		
					tft_prints(0, 5, "Never plug to");
					tft_prints(0, 6, "other vcc");
					tft_prints(0, 7, "or ground!");
					tft_prints(0, 8, "Suggest use usb");
					tft_prints(0, 9, "power to test!");
				} else {
					tft_prints(0, 1, "Pin Test");
					tft_prints(0, 2, "Done!");
					tft_prints(0, 3, "Unplug test wire");
					tft_prints(0, 4, "Press centre");
					tft_prints(0, 5, "to quit");
				}
				tft_update();
			}
		}
	}
}
/** usart test ***/
static void uart1_rx_test_handler(u8 rx_data)
{	
	received_data[0]= rx_data;
}
static void uart2_rx_test_handler(u8 rx_data)
{	
	received_data[1]= rx_data;
}
static void uart3_rx_test_handler(u8 rx_data)
{	
	received_data[2]= rx_data;
}
static void uart4_rx_test_handler(u8 rx_data)
{	
	received_data[3]= rx_data;
}
static void uart5_rx_test_handler(u8 rx_data)
{	
	received_data[4]= rx_data;
}

static void uart_on_click_listener(TFT_UI_ITEM** uart_item)
{
  uart_tx_byte((COM_TypeDef) tft_ui_get_val(uart_item[0]),tft_ui_get_val(uart_item[1]));
  CLICK_MUSIC;
}

void uart_test(void)
{
	//init tx and rx
	uart_init(COM1,115200);
  uart_init(COM2,115200);
  uart_init(COM3,115200);
	uart_init(COM4,115200);
	uart_init(COM5,115200);
	uart_rx_init(COM1, uart1_rx_test_handler);
  uart_rx_init(COM2, uart2_rx_test_handler);
  uart_rx_init(COM3, uart3_rx_test_handler);
	uart_rx_init(COM4, uart4_rx_test_handler);
	uart_rx_init(COM5, uart5_rx_test_handler);
  
  TFT_UI_ITEM
    uart_tx_port = {
      .type = tft_ui_list,
      .x = 6, .y = 2,
      .ui_item.list = {
        .width = 7,
        .range.lower = (s32) COM1,
        .range.upper = (s32) COM5,
        .selected_int = COM1
      }
    },
    
    uart_tx_data = {
      .type = tft_ui_list,
      .x = 8, .y = 3,
      .ui_item.list = {
        .width = 5,
        .range.lower = 0,
        .range.upper = 255,
        .selected_int = 65
      }
    },
    
    uart_rx_port = {
      .type = tft_ui_list,
      .x = 6, .y = 4,
      .ui_item.list = {
        .width = 7,
        .range.lower = (s32) COM1,
        .range.upper = (s32) COM5,
        .selected_int = COM1
      }
    };
    
  TFT_UI_ITEM* uart_test_ui_list[] = {
		&uart_tx_port, &uart_tx_data, &uart_rx_port
	};
    
  TFT_UI tft_ui = {
		.item_count = 3, .item_list = uart_test_ui_list, .selected_item = 0, .click_event = uart_on_click_listener
	};    
  
  while (true) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      if (ticks_img % 50 == 0) {
        battery_adc_update();
      }
      
      if (ticks_img % 50 == 3) {
        button_update();
        if (return_listener()) {
          return;
        }
        
        if (button_pressed(BUTTON_JS2_UP) == 1) {				
          tft_ui_listener(&tft_ui, tft_ui_event_up);
        }
        
        if (button_pressed(BUTTON_JS2_DOWN) == 1) {
          tft_ui_listener(&tft_ui, tft_ui_event_down);
        }
        
        if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_left);
				}
				
				if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 2)) {
					tft_ui_listener(&tft_ui, tft_ui_event_right);
				}
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					tft_ui_listener(&tft_ui, tft_ui_event_select);
				}        
      }
      
      if (ticks_img % 50 == 6) {
        tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "UART TEST");
				tft_prints(0, 2, "Tx: ");
        tft_prints(8, 2, "UART%d", tft_ui_get_val(&uart_tx_port) + 1);
				tft_prints(0, 3, "Tx data:");
        tft_prints(11, 3, "%c", tft_ui_get_val(&uart_tx_data));
				tft_prints(0, 4, "Rx: ");
        tft_prints(8, 4, "UART%d", tft_ui_get_val(&uart_rx_port) + 1);
				tft_prints(0, 5, "Rx data:");
        tft_prints(11, 5, "%c ", received_data[tft_ui_get_val(&uart_rx_port)]);
				if (ticks_img < 500) {
					tft_prints(0, 7, "Click to send");
				}
        tft_ui_update(&tft_ui, ticks_img % 500 < 25);
				tft_update();
      }
    }
  }
}

void ultra_test(void)
{
  //u16 distance_history[tft_width-2] = {0};

  while (true) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      if (ticks_img % 50 == 0) {
        battery_adc_update();
      }
      
      if (ticks_img % 50 == 3) {
        button_update();
        if (return_listener()) {
          return;
        }
      }
      
      if (ticks_img % 50 == 6) {
        tft_clear();
        draw_top_bar();
        tft_prints(0, 1, "ULTRA. TEST");
        tft_prints(0, 2, " %5d/%5d ", ultrasonic_get_successful_count(), ultrasonic_get_count());
        tft_prints(0, 3, "Pulse: %d", get_pulse_width());
        tft_prints(0, 4, "Distance: %d", get_distance());        
        tft_prints(0, 5, "Avg.: %d", ultrasonic_get_distance_avg());
        tft_update();
        
        
      }
    }
  }
}

