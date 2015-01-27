#include "system_test.h"

static u16 ticks_img 	= (u16)-1;
static u8 received_data[5] = {0};

static u8 return_listener(void)
{
	return button_pressed(BUTTON_1) > 10 || button_pressed(BUTTON_2) > 10;
}

void battery_test(void)
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
				
				static u8 char_start = STARTING_ASCII;
				
				if (button_pressed(BUTTON_JS2_DOWN) == 1 || button_pressed(BUTTON_JS2_UP) == 1) {
					char_start = (char_start == STARTING_ASCII) ? STARTING_ASCII + tft_width * (tft_height - 2) : STARTING_ASCII;
					CLICK_MUSIC;
				}
				char char_i = char_start; 
				/** Display characters from ascii value 0 **/
				for (u8 y = 2; y < tft_height; ++y) {
					for (u8 x = 0; x < tft_width; ++x) {
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
				.width = 11,
				.range.lower = 0,
				.range.upper = CAN_MOTOR_COUNT-1,
				.selected_int = 0
			}	
		},
		
		motor_loop_type = {
			.type = tft_ui_list,
			.x = 2, .y = 5,
			.ui_item.list = {
				.width = 11,
				.range.lower = (s32) 0,
				.range.upper = (s32) 1,
				.selected_int = 0
			}
		},
		
		motor_speed = {
			.type = tft_ui_list,
			.x = 7, .y = 6,
			.ui_item.list = {
				.width = 7,
				.range.lower = (s32) -500,
				.range.upper = (s32) 500,
				.selected_int = 0
			}
		},
		
		motor_test_flag = {
			.type = tft_ui_checkbox,
			.x = 1, .y = 7,
			.ui_item.checkbox = {
				.checked = false
			}
		}
	;  
	
	TFT_UI_ITEM* motor_test_ui_list[] = {
		&motor_list, &motor_loop_type, &motor_speed, &motor_test_flag
	}; 
	
	TFT_UI tft_ui = {
		4, motor_test_ui_list, 0
	};
	
	while (1) {
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
				
				if (button_pressed(BUTTON_JS2_CENTER) == 1) {
					gyro_cal();
					CLICK_MUSIC;
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "POSITION TEST");
				tft_prints(0, 2, " X: %d", get_pos()->x);
				tft_prints(0, 3, " Y: %d", get_pos()->y);
				tft_prints(0, 4, " A: %d", get_pos()->angle);
				tft_prints(0, 5, " Avail: %d", gyro_available);
				
				tft_prints(0, 7, " (%c) Calibrate", ticks_img < 500 ? BLACK_BLOCK_ASCII : ' ');
				
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
				for (u8 i = 0, x = 1, y = 2; i < BUTTON_COUNT; ++i) {
					tft_prints(x, y, "%d:%d", i, button_pressed((BUTTON) i));
					if (y < tft_height - 1) {
						++y;
					} else {
						x += 6;
						y = 2;
					}
				}
				tft_update();
			}
		}
	}	
}

void buzzer_test(void)
{
	const u8 line_available[] = {2, 3, 4, 5};
	u8 line_id = 0;
	u16 duration = 10;
	u8 volume = 30;
	MUSIC_NOTE note[] = {{NOTE_G, 7}, {NOTE_END}};
	const char* note_char[] = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B"};
	while (true) {
		if (ticks_img != get_ticks()) {			
			ticks_img = get_ticks();			
			
			u8 line = line_available[line_id];
			
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
				if (button_pressed(BUTTON_JS2_UP) == 1) {
					if (line_id == 0) {
						line_id = sizeof(line_available) / sizeof(u8) - 1;
					} else {
						--line_id;
					}
				}
				if (button_pressed(BUTTON_JS2_DOWN) == 1) {
					line_id = (line_id + 1) % (sizeof(line_available) / sizeof(u8));
				}
				// Line
				switch (line) {
					// Pitch setting
					case 2:
						if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
							if (note[0].note > NOTE_C) {
								--note[0].note;
							} else if (note[0].octave > 1) {
								--note[0].octave;
								note[0].note = NOTE_B;
							}			
						}							
						if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
							if (note[0].note < NOTE_B) {
								++note[0].note;
							} else if (note[0].octave < 10) {
								note[0].note = NOTE_C;
								++note[0].octave;
							}
						}
					break;
					// Octave setting
					case 3:
						if (button_pressed(BUTTON_JS2_LEFT) == 1) {
							if (note[0].octave > 1) {
								--note[0].octave;
							}			
						}							
						if (button_pressed(BUTTON_JS2_RIGHT) == 1) {
							if (note[0].octave < 10) {
								++note[0].octave;
							}
						}
						break;
					// Volume setting
					case 4:
						if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
							if (volume > 0) {
								--volume;
							}
						}						
						if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
							if (volume < 100) {
								++volume;
							}
						}
						break;
					// Duration setting
					case 5:
						if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
							if (duration > 10) {
								duration -= 10;
							}
						}						
						if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
							if (duration < 65535) {
								duration += 10;
							}
						}		
          break;
        }
        // Click to play
        if (button_pressed(BUTTON_JS2_CENTER) == 1) {
          buzzer_stop_song();										// stop last note
          buzzer_set_volume(volume);						// set with given volume
          buzzer_play_song(note, duration, 0);	// Play the note.						
        }
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "BUZZER TEST");
				tft_prints(0, 2, " Notes: ");
				if (ticks_img < 500 || line != 2) {
					tft_prints(11, 2, "%s", note_char[note[0].note - 1]);
				}
				tft_prints(0, 3, " Octave");
				if (ticks_img < 500 || line != 3) {
					tft_prints(11, 3, "%d", note[0].octave);
				}
				tft_prints(0, 4, " Volume: ");
				if (ticks_img < 500 || line != 4) {
					tft_prints(11, 4, "%d", volume);
				}
				tft_prints(0, 5, " Duration:");
				if (ticks_img < 500 || line != 5) {
					tft_prints(11, 5, "%d", duration);
				}
				if (ticks_img < 500) {
					tft_prints(1, 7, "Click to play");
				}
				tft_update();
			}
		}
	}
}

void can_test(void)
{
  const u8 line_available[] = {2, 3};
  u8 line_id = 0;
  u8 sent_id = 0;
  u8 tx_length = 1;
  u16 sent = 0;
  while (true) {
    if (ticks_img != get_ticks()) {
      ticks_img = get_ticks();
      u8 line = line_available[line_id];
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
          if (line_id == 0) {
            line_id = sizeof(line_available) / sizeof(u8) - 1;
          } else {
            --line_id;
          }
        }
        if (button_pressed(BUTTON_JS2_DOWN) == 1) {
          line_id = (line_id + 1) % (sizeof(line_available) / sizeof(u8));
        }
        // Line
        switch (line) {
          case 2:
            if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
              --sent_id;
            }
            if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
              ++sent_id;
            }
          break;
          case 3:
            if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
              if (tx_length > 1) {
                --tx_length;
              }
            }
            if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1)) {
              if (tx_length < 7) {
                ++tx_length;
              }
            }
          break;
        }
        if (button_pressed(BUTTON_JS2_CENTER) == 1) {
          CAN_MESSAGE txmsg;
          txmsg.id = sent_id;
          txmsg.length = tx_length;
          for (u8 i = 0; i < txmsg.length; ++i) {
            txmsg.data[i] = 0x00;
          }
          can_tx_enqueue(txmsg);
          ++sent;
          CLICK_MUSIC;
        }
      }
      if (ticks_img % 50 == 6) {
        tft_clear();
        draw_top_bar();
        tft_prints(0, 1, "CAN TEST");
        tft_prints(0, 2, "CAN id:");
        if (ticks_img < 500 || line != 2) {
          tft_prints(11, 2, "00 %02X", sent_id);
        }
        tft_prints(0, 3, "CAN length:");
        if (ticks_img < 500 || line != 3) {
          tft_prints(11, 3, "%d", tx_length);
        }
        tft_prints(0, 4, "Sent no.: %d", sent);
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
        tft_update();
      }
    }
  }
}

void xbc_test(void)
{
	u8	pressed_cnt = 0;
	u8  pre_pressed_cnt = 0;
	u16 press_times = 0;
	u8 xbc_test_ko = 0;
	u32 pre_xbc_press = 0;
	while(!xbc_test_ko) {
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
				pre_pressed_cnt = pressed_cnt;
				pre_xbc_press = xbc_press;
				xbc_update();
				tft_clear();
				switch (pressed_cnt) {
					case 0:	
						tft_prints(0,2,"PRESS [START]");
						if (xbc_press & XBC_START) {
							pressed_cnt++;
						}
					break;

					case 1:
						tft_prints(0,2,"PRESS [UP]");
						if (xbc_press & XBC_UP) {
							pressed_cnt++;
						}
					break;

					case 2:
						tft_prints(0,2,"PRESS [DOWN]   ");
						if (xbc_press & XBC_DOWN) {
							pressed_cnt++;
						}
					break;

					case 3:
						tft_prints(0,2,"PRESS [LEFT]   ");
						if (xbc_press & XBC_LEFT) {
							pressed_cnt++;
						}
					break;

					case 4:
						tft_prints(0,2,"PRESS [RIGHT]   ");
						if (xbc_press & XBC_RIGHT) {
							pressed_cnt++;
						}
					break;

					case 5:
						tft_prints(0,2,"PRESS [BACK]   ");
						if (xbc_press & XBC_BACK) {
							pressed_cnt++;
						}
					break;

					case 6:
						tft_prints(0,2,"PRESS [A]     ");
						if (xbc_press & XBC_A) {
							pressed_cnt++;
						}
					break;

					case 7:
						tft_prints(0,2,"PRESS [B]    ");
						if (xbc_press & XBC_B) {
							pressed_cnt++;
						}
					break;

					case 8:
						tft_prints(0,2,"PRESS [X]    ");
						if (xbc_press & XBC_X) {
							pressed_cnt++;
						}
					break;

					case 9:
						tft_prints(0,2,"PRESS [Y]     ");
						if (xbc_press & XBC_Y) {
							pressed_cnt++;
						}
					break;

					case 10:
						tft_prints(0,2,"PRESS [LB ]   ");
						if (xbc_press & XBC_LB) {
							pressed_cnt++;
						}
					break;

					case 11:
						tft_prints(0,2,"PRESS [RB ]   ");
						if (xbc_press & XBC_RB) {
							pressed_cnt++;
						}
					break;

					case 12:
						tft_prints(0,2,"PRESS [XBOX  ]   ");
						if (xbc_press & XBC_XBOX) {
							pressed_cnt++;
						}
					break;

					case 13:
						tft_prints(0,2,"[L BUTTON 1]");
						if (xbc_press & L_BUT1) {
							pressed_cnt++;
						}
					break;

					case 14:
						tft_prints(0,2,"[L BUTTON 2]");
						if (xbc_press & L_BUT2) {
							pressed_cnt++;
						}
					break;

					case 15:
						tft_prints(0,2,"[L BUTTON 3]");
						if (xbc_press & L_BUT3) {
							pressed_cnt++;
						}
					break;


					case 16:
						tft_prints(0,2,"[R BUTTON 1]");
						if (xbc_press & R_BUT1) {
							pressed_cnt++;
						}
					break;

					case 17:
						tft_prints(0,2,"[R BUTTON 2]");
						if (xbc_press & R_BUT2) {
							pressed_cnt++;
						}
					break;

					case 18:
						tft_prints(0,2,"[R BUTTON 3]");
						if (xbc_press & R_BUT3) {
							pressed_cnt++;
						}
					break;

					case 19:
						tft_prints(0,2,"press [BACK] exit");
						tft_prints(0,3,"KEY TEST OK");
						if (xbc_press & XBC_BACK) {
							xbc_test_ko = 1;
						}
				}

				if (pressed_cnt != pre_pressed_cnt) {
					//buzzer on when press right button
					SUCCESSFUL_MUSIC;
				}

				if (xbc_press != pre_xbc_press && xbc_press != 0) {
					press_times++;
				}
				draw_top_bar();
				tft_prints(0,1,"XBOX TEST");
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
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
	u16 curr_test_pin = GPIO_Pin_0;
	vu16 initial_input = 0;		// None of GPIOE is used before test
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
					if ((GPIO_ReadInputData(GPIOE) - initial_input) == curr_test_pin && last_input == initial_input) {
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
void uart1_rx_handler(u8 rx_data){	
	received_data[0]= rx_data;
}
void uart2_rx_handler(u8 rx_data){	
	received_data[1]= rx_data;
}
void uart3_rx_handler(u8 rx_data){	
	received_data[2]= rx_data;
}
void uart4_rx_handler(u8 rx_data){	
	received_data[3]= rx_data;
}
void uart5_rx_handler(u8 rx_data){	
	received_data[4]= rx_data;
}
void uart_test(void)
{
	//init tx and rx
	uart_init(COM1,115200);
	uart_init(COM4,115200);
	uart_init(COM5,115200);
	uart_rx_init(COM1, uart1_rx_handler);
	uart_rx_init(COM4, uart4_rx_handler);
	uart_rx_init(COM5, uart5_rx_handler);
	u8 line_id = 0;
	const u8 line_available[] = {2, 3, 4};
  const u8 tx_id = 0, rx_id = 1;
  u8 COM_id[2] = {0, 1};                                    // Tx and Rx, Rx is default as 1 after tx
  const COM_TypeDef usart_available[] = {COM1, COM4, COM5}; // Make sure at least two available
	u8 sent_data = 65;
  
	while (!return_listener()) {
		if (ticks_img != get_ticks()) {
			ticks_img = get_ticks();
			u8 line = line_available[line_id];
      COM_TypeDef COMx[2] = {usart_available[COM_id[tx_id]], usart_available[COM_id[rx_id]]};
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
					if (line_id == 0) {
						line_id = sizeof(line_available) / sizeof(u8) - 1;
					} else {
						--line_id;
					}
				}
				if (button_pressed(BUTTON_JS2_DOWN) == 1) {
					line_id = (line_id + 1) % (sizeof(line_available) / sizeof(u8));
				}
				// Line
				switch(line){
					case 2:
						if (button_pressed(BUTTON_JS2_LEFT) == 1) {
              if (COM_id[tx_id] == 0) {
                COM_id[tx_id] = sizeof(usart_available) / sizeof(u8) - 1;
              } else {
                --COM_id[tx_id];
              }
            }
            if (button_pressed(BUTTON_JS2_RIGHT) == 1) {
              COM_id[tx_id] = (COM_id[tx_id] + 1) % (sizeof(usart_available) / sizeof(u8));
						}
					break;
					case 3:
						if (button_pressed(BUTTON_JS2_LEFT) == 1 || button_hold(BUTTON_JS2_LEFT, 10, 1)) {
							--sent_data;
						} else if (button_pressed(BUTTON_JS2_RIGHT) == 1 || button_hold(BUTTON_JS2_RIGHT, 10, 1) ) {
							++sent_data;
						}					
						break;
					case 4:
						if (button_pressed(BUTTON_JS2_LEFT) == 1) {
              if (COM_id[rx_id] == 0) {
                COM_id[rx_id] = sizeof(usart_available) / sizeof(u8) - 1;
              } else {
                --COM_id[rx_id];
              }
						} else if (button_pressed(BUTTON_JS2_RIGHT) == 1) {
              COM_id[rx_id] = (COM_id[rx_id] + 1) % (sizeof(usart_available) / sizeof(u8));
						}
					break;
				}
				if (button_pressed(BUTTON_JS2_CENTER) == 3) {
					uart_tx_byte(COMx[tx_id],sent_data);
					CLICK_MUSIC;					
				}
			}				
			if(ticks_img % 50 == 9){	
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "UART TEST");
				tft_prints(0, 2, "Tx: ");
				if (ticks_img < 500 || line != 2) {
					tft_prints(6, 2, "USART%d", COMx[tx_id] + 1);
				}
				tft_prints(0, 3, "Tx data");
				if (ticks_img < 500 || line != 3) {
					tft_prints(11, 3, "%c", sent_data);
				}
				tft_prints(0, 4, "Rx: ");
				if (ticks_img < 500 || line != 4) {
					tft_prints(6, 4, "USART%d", COMx[rx_id] + 1);
				}
				tft_prints(0, 5, "Rx_data: %c ", received_data[COMx[rx_id]]);
				if (ticks_img < 500) {
					tft_prints(0, 7, "Click to send");
				}
				tft_update();
			}	
		}
	}
}
