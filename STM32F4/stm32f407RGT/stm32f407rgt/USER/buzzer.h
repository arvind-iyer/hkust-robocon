#ifndef BUZZER_H
#define BUZZER_H


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "gpio.h"
#include "timer.h"

#define BUZZER_GPIO  				&PE6

#define BUZZER_TIM                  TIM9
#define BUZZER_TIM_RCC              RCC_APB2Periph_TIM9
#define BUZZER_CHANNEL         		TIM_Channel_2

#define BUZZER_TIM_OC_INIT          TIM_OC2Init
#define BUZZER_TIM_SETCOMPARE       TIM_SetCompare2

#define BUZZER_COUNT_PER_SECOND     1000000     /*!< Buzzer timer period, used for prescaling and relavant calculation */

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);

void buzzer_control(u8 count, u16 period);
void buzzer_check(void);
void buzzer_set_volume(u8 vol); //0-100

// Note frequency (enharmonic equivalent notes have the same value)
typedef enum {
	NOTE_END		=   -1,
	NOTE_REST		=	0,
	NOTE_C			=   1,    // C
	NOTE_Cs			=	2,    // C# 
	NOTE_Db			=	2,    // Db
	NOTE_D			=	3,    // D
	NOTE_Ds			=	4,    // D#
	NOTE_Eb			=	4,    // Eb
	NOTE_E			=	5,    // E
	NOTE_Fb			=	5,    // Fb
	NOTE_F			=	6,    // F
	NOTE_Fs			=	7,    // Fs
	NOTE_Gb			=	7,    // Gb
	NOTE_G			=	8,    // G
	NOTE_Gs			=	9,    // G#
	NOTE_Ab			=	9,    // Ab
	NOTE_A			=	10,   // A
	NOTE_As			=	11,   // A#
	NOTE_Bb			=	11,   // Bb
	NOTE_B			=	12,   // B
	NOTE_Bs			=	1     // B#
} MUSIC_NOTE_LETTER;


#define C0_PERIOD                           61158
#define TWELFTH_ROOT_OF_TWOx10000     10595


typedef struct {
	MUSIC_NOTE_LETTER note;
	u8 octave;
} MUSIC_NOTE;


#endif	// BUZZER_H
	


void buzzer_set_note_period(u16 p);
void buzzer_set_volume(u8 vol); // 0 - 100 (0: muted, 100: full)
u16 get_note_period(MUSIC_NOTE_LETTER note, u8 octave);


void buzzer_play_song(const MUSIC_NOTE* song, u16 note_length, u16 note_break);
void buzzer_stop_song(void);

