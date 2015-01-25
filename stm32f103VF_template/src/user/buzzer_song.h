#ifndef	__BUZZER_SONG_H
#define	__BUZZER_SONG_H

#include "buzzer.h"

extern const MUSIC_NOTE START_UP[];
extern const MUSIC_NOTE SUCCESSFUL_SOUND[];
extern const MUSIC_NOTE FAIL_SOUND[];
extern const MUSIC_NOTE BIRTHDAY_SONG[];
extern const MUSIC_NOTE CLICK[];

#define	SUCCESSFUL_MUSIC			buzzer_play_song(SUCCESSFUL_SOUND, 100, 0);
#define	FAIL_MUSIC 						buzzer_play_song(FAIL_SOUND, 120, 100);
#define	CLICK_MUSIC						buzzer_play_song(CLICK, 120, 0);
#endif /* __BUZZER_SONG_H */
