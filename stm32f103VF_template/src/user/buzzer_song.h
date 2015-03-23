#ifndef	__BUZZER_SONG_H
#define	__BUZZER_SONG_H

#include "buzzer.h"

extern const MUSIC_NOTE START_UP[];
extern const MUSIC_NOTE SUCCESSFUL_SOUND[];
extern const MUSIC_NOTE FAIL_SOUND[];
extern const MUSIC_NOTE BIRTHDAY_SONG[];
extern const MUSIC_NOTE CLICK[];

extern const MUSIC_NOTE SPEED_0[];
extern const MUSIC_NOTE SPEED_1[];
extern const MUSIC_NOTE SPEED_2[];
extern const MUSIC_NOTE SPEED_3[];
extern const MUSIC_NOTE SPEED_4[];
extern const MUSIC_NOTE SPEED_5[];
extern const MUSIC_NOTE SPEED_6[];
extern const MUSIC_NOTE SPEED_7[];
extern const MUSIC_NOTE SPEED_8[];
extern const MUSIC_NOTE SPEED_9[];

#define	SUCCESSFUL_MUSIC			buzzer_play_song(SUCCESSFUL_SOUND, 100, 0)
#define	FAIL_MUSIC 						buzzer_play_song(FAIL_SOUND, 120, 100)
#define	CLICK_MUSIC						buzzer_play_song(CLICK, 50, 0)
#endif /* __BUZZER_SONG_H */
