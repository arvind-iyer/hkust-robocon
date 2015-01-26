#include "system_test.h"

static u16 ticks_img 	= (u16)-1;
static u16 seconds_img = (u16)-1;

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
				
				char c = 0;
				
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
				tft_prints(5, 2, "MOTOR%-2d", tft_ui_get_val(&motor_list));
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
					tft_prints(x, y, "%d:%d", i, button_pressed(i));
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
	const u8 line_available[] = {2, 3, 4, 5, 7};
	u8 line_id = 0;
	static u16 duration = 10;
	static u8 volume = 30;
	static MUSIC_NOTE note[] = {{NOTE_G, 7}, {NOTE_END}};
	const char* note_char[] = {"C", "Cs", "D", "Eb", "E", "F", "Fs", "G", "Gs", "A", "Bb", "B"};
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
					// Click to play
					case 7:
						if (button_pressed(BUTTON_JS2_CENTER) == 1) {
							buzzer_stop_song();										// stop last note
							buzzer_set_volume(volume);						// set with given volume
							buzzer_play_song(note, duration, 0);	// Play the note.						
						}
						break;
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
				if (line != 7) {
					tft_prints(0, 7, "Play test");
				} else if (ticks_img < 500) {
					tft_prints(0, 7, "Click to play");
				}
				tft_update();
			}
		}
	}
}

void can_test(void)
{
	const u8 line_available[] = {2, 3, 9};
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
					case 9:
						if (button_pressed(BUTTON_JS2_CENTER) == 1) {
							CAN_MESSAGE msg;
							msg.id = sent_id;
							msg.length = tx_length;
							for (u8 i = 0; i < msg.length; ++i) {
								msg.data[i] = 0x00;
							}
							can_tx_enqueue(msg);
							++sent;
							CLICK_MUSIC;
						}
						break;
				}
			}
			
			if (ticks_img % 50 == 6) {
				tft_clear();
				draw_top_bar();
				tft_prints(0, 1, "CAN TEST");
				tft_prints(0, 2, "CAN id:");
				if (ticks_img < 500 || line != 2) {
					tft_prints(11, 2, "00 %02x", sent_id);
				}
				tft_prints(0, 3, "CAN length:");
				if (ticks_img < 500 || line != 3) {
					tft_prints(11, 3, "%d", tx_length);
				}
				tft_prints(0, 7, "Sent: %d", sent);
				if (line != 9){
					tft_prints(5, 9, "SEND");
				} else if (ticks_img < 500) {
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
				if (button_pressed(BUTTON_2) == 1 && pressed_cnt > 0) {
					--pressed_cnt;
				}
				if (button_pressed(BUTTON_1) == 1 && pressed_cnt < 19) {
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
					buzzer_control(2,5);
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

