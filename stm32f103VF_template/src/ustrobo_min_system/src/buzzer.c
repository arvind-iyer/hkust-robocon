#include "buzzer.h"


// Buzzer control related
u8 buzzer_on_flag = 0;
u16 buzzer_period = 0;
u16 buzzer_time_ms = 0;		// Period left
u8 buzzer_count = 0;

// Note frequency related
static u8 buzzer_volume = 30;		// 0 - 100 (101 for full buzz)
static u16 buzzer_note_period = 1;

// Song related
u8 buzzer_song_flag = 0;	// 1 if a song is being played
const MUSIC_NOTE* buzzer_current_song = 0;
u16 buzzer_current_song_note_id = 0;
u16 buzzer_song_note_length = 0;
u16 buzzer_song_note_length_left = 0;// Length left
u16 buzzer_song_note_break = 0;	// break length between notes (can be 0)
u8 buzzer_song_note_break_flag = 0;

/**
  * @brief  Initialization of buzzer
  * @param  None
  * @retval None
  */
void buzzer_init(void)
{		   	
	GPIO_InitTypeDef BUZZER_InitStructure; 			
	RCC_APB2PeriphClockCmd(BUZZER_RCC, ENABLE);
	BUZZER_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			   
	BUZZER_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	BUZZER_InitStructure.GPIO_Pin = BUZZER_PIN;
	
	
	// buzzer frequency init
	
	{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;      // TimeBase is for timer setting   > refer to P. 344 of library
		TIM_OCInitTypeDef  TIM_OCInitStructure;             // OC is for channel setting within a timer  > refer to P. 342 of library

		
		RCC_APB1PeriphClockCmd(BUZZER_TIM_RCC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//GPIO_PinRemapConfig(BUZZER_TIM_REMAP, ENABLE);
		
		
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // counter will count up (from 0 to FFFF)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;       //timer clock = dead-time and sampling clock 	
		TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / BUZZER_COUNT_PER_SECOND - 1;                         // 1MHz
		TIM_TimeBaseStructure.TIM_Period = buzzer_note_period;	                    

		
		TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);       // this part feeds the parameter we set above
		
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         //set "high" to be effective output
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	             //produce output when counter < CCR
		
		TIM_OCInitStructure.TIM_Pulse = buzzer_volume;
		
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				
		TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
		TIM_Cmd(BUZZER_TIM, ENABLE);	
		
		//TIM_OC1Init(BUZZER_TIM, &TIM_OCInitStructure);
		BUZZER_TIM_OC_INIT(BUZZER_TIM, &TIM_OCInitStructure);
		//TIM_OC3Init(BUZZER_TIM, &TIM_OCInitStructure);
		//TIM_OC4Init(BUZZER_TIM, &TIM_OCInitStructure);
	
	}
	
	
	GPIO_Init(BUZZER_PORT, &BUZZER_InitStructure); 
	buzzer_off();
	
	//GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
	

}


void buzzer_on(void)
{
	//GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
	if (buzzer_note_period == 0) {
		buzzer_off();
	} else {
		BUZZER_TIM_SETCOMPARE(BUZZER_TIM, buzzer_note_period * buzzer_volume / 100);
	}
	
}

void buzzer_off(void)
{
	//GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
	BUZZER_TIM_SETCOMPARE(BUZZER_TIM, 0);
}




/**
  * @brief  Buzzer check per tick
  * @param  None
  * @retval None
  */
void buzzer_check(void)
{
	if (buzzer_on_flag > 0 || buzzer_count > 0) {
		--buzzer_time_ms;
		if (buzzer_time_ms == 0) {
			buzzer_on_flag = !buzzer_on_flag;
			buzzer_on_flag ? buzzer_on() : buzzer_off();

			buzzer_time_ms = buzzer_period;
			
			if (!buzzer_on_flag) {
				--buzzer_count;
			}
		}
	} else if (buzzer_song_flag) {
		MUSIC_NOTE current_note = buzzer_current_song[buzzer_current_song_note_id];
		
		// Check if the song is ended
		if (current_note.note == NOTE_END) {
			buzzer_song_flag = 0;
			buzzer_off();
		}
		
		
		if (buzzer_song_note_length_left > 0) {
			--buzzer_song_note_length_left;
		} else {
			buzzer_song_note_length_left = buzzer_song_note_length;
			if (buzzer_song_note_break) {
				buzzer_song_note_break_flag = !buzzer_song_note_break_flag;
				if (buzzer_song_note_break_flag) {
					buzzer_song_note_length_left = buzzer_song_note_break;
					buzzer_off();
				} else {
					buzzer_on();
				}
			}

			// Change note
			if ((buzzer_song_note_break && !buzzer_song_note_break_flag) || buzzer_song_note_break == 0) {
				current_note = buzzer_current_song[++buzzer_current_song_note_id];
				buzzer_set_note_period(get_note_period(current_note.note, current_note.octave));
				buzzer_on();
			}

			
		}
	}
}




/**
  * @brief  Generate specific pattern of buzzer
  * @param  count: number of buzz to be generated
  * @param  period: time for each buzz (ms)
  * @retval None
  */
void buzzer_control(u8 count, u16 period)
{
	if (count == 0 || period == 0) {return;}	// Do nothing

	buzzer_count = count;
	buzzer_period = buzzer_time_ms = period;
	buzzer_on_flag = 1;
	buzzer_on();
	
	buzzer_stop_song();	// Cut current playing song 
}




void buzzer_set_note_period(u16 p)
{
	buzzer_note_period = p;
	if (buzzer_note_period > 0) {
		TIM_SetAutoreload(BUZZER_TIM, buzzer_note_period);
	} else {
		buzzer_off();
	}
	//BUZZER_TIM_SETCOMPARE(BUZZER_TIM, BUZZER_COUNT_PER_SECOND / freq * buzzer_volume / 100);
}

void buzzer_set_volume(u8 vol)
{
	if (vol > 100) {vol = 100;}
	buzzer_volume = vol;
}



u16 get_note_period(MUSIC_NOTE_LETTER note, u8 octave)
{
	static const u16 NOTE0_PERIOD[13] = {
		0,	C0_PERIOD, Db0_PERIOD, D0_PERIOD, Eb0_PERIOD, E0_PERIOD, F0_PERIOD, 
		Gb0_PERIOD, G0_PERIOD, Ab0_PERIOD, A0_PERIOD, Bb0_PERIOD, B0_PERIOD
	};
	u16 note_period = 0;
	if (note == NOTE_REST || note == NOTE_END || note > 12) return 0;
	
	note_period = NOTE0_PERIOD[note];
	
	while (octave--) {
		note_period /= 2;		// One octave higher ~= period / 2
	}
	
	return note_period;
	
}


void buzzer_play_song(const MUSIC_NOTE* song, u16 note_length, u16 note_break)
{
	if (song == 0) {return;}
	// Cut buzzer_control
	buzzer_on_flag = 0;
	buzzer_count = 0;
	
	buzzer_song_flag = 1;
	buzzer_current_song_note_id = 0;
	buzzer_song_note_length = note_length;
	buzzer_song_note_length_left = note_length;
	buzzer_song_note_break = note_break;
	buzzer_song_note_break_flag = 0;
	
	buzzer_current_song = song;
	buzzer_set_note_period(get_note_period(song[0].note, song[0].octave));
	buzzer_on();
	
}

void buzzer_stop_song(void)
{
	buzzer_song_flag = 0;
}
	
