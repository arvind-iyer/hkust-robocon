#ifndef	__BUZZER_H
#define	__BUZZER_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define BUZZER_PORT							GPIOB
#define BUZZER_PIN							GPIO_Pin_0	
#define BUZZER_RCC							RCC_APB2Periph_GPIOB

#define BUZZER_TIM							TIM3
#define BUZZER_TIM_RCC					RCC_APB1Periph_TIM3
#define BUZZER_TIM_REMAP				GPIO_FullRemap_TIM3

#define BUZZER_TIM_OC_INIT			TIM_OC3Init
#define	BUZZER_TIM_SETCOMPARE		TIM_SetCompare3

#define BUZZER_COUNT_PER_SECOND		1000000		// Used for prescaling and relavant calculation

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);

void buzzer_control(u8 count, u16 period);
void buzzer_check(void);


// Note frequency 
typedef enum {
	NOTE_END		= -1,
	NOTE_REST		=	0,
	NOTE_C			= 1,
	NOTE_Cs			=	2,
	NOTE_Db			=	2,
	NOTE_D			=	3,
	NOTE_Ds			=	4,
	NOTE_Eb			=	4,
	NOTE_E			=	5,
	NOTE_Fb			=	5,
	NOTE_F			=	6,
	NOTE_Fs			=	7,
	NOTE_Gb			=	7,
	NOTE_G			=	8,
	NOTE_Gs			=	9,
	NOTE_Ab			=	9,
	NOTE_A			=	10,
	NOTE_As			=	11,
	NOTE_Bb			=	11,
	NOTE_B			=	12,
	NOTE_Bs			=	1
} MUSIC_NOTE_LETTER;

// Sound period (1/freq) of the 0th octave in milliseconds (us)
#define	C0_PERIOD			61158		
#define	Db0_PERIOD		57725
#define	D0_PERIOD			54485
#define	Eb0_PERIOD		51426
#define	E0_PERIOD			48540
#define	F0_PERIOD			45816
#define	Gb0_PERIOD		43245
#define	G0_PERIOD			40818
#define	Ab0_PERIOD		38527
#define	A0_PERIOD			36364
#define	Bb0_PERIOD		34323
#define	B0_PERIOD			32397


typedef struct {
	MUSIC_NOTE_LETTER note;
	u8 octave;
} MUSIC_NOTE;


static MUSIC_NOTE START_UP[] = {{NOTE_G,6},{NOTE_B,6},{NOTE_D,7},{NOTE_G,7},{NOTE_G,7},{NOTE_END,2}};


void buzzer_set_note_period(u16 p);
void buzzer_set_volume(u8 vol);	// 0 - 100 (0: muted, 100: full)
u16 get_note_period(MUSIC_NOTE_LETTER note, u8 octave);


void buzzer_play_song(MUSIC_NOTE* song, u16 note_length, u16 note_break);
void buzzer_stop_song(void);

#endif	/* __BUZZER_H */
