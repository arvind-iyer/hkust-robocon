#ifndef	__BUZZER_SONG_H
#define	__BUZZER_SONG_H

#include "buzzer.h"

extern const MUSIC_NOTE START_UP[];
extern const MUSIC_NOTE SUCCESSFUL_SOUND[];
extern const MUSIC_NOTE FAIL_MUSIC1[];
extern const MUSIC_NOTE BIRTHDAY_SONG[];
extern const MUSIC_NOTE CLICK[];
extern const MUSIC_NOTE CONNECTED[];
extern const MUSIC_NOTE DISCONNECTED[];

extern const MUSIC_NOTE FAIL_MUSIC2[];

extern const MUSIC_NOTE MARIO_BEGIN[];
extern const MUSIC_NOTE MARIO_END[];
extern const MUSIC_NOTE	OKAY_MUSIC[];
extern const MUSIC_NOTE OKAY_MUSIC2[];

#define	SUCCESSFUL_MUSIC			buzzer_play_song(SUCCESSFUL_SOUND, 100, 0)
#define	PLAY_FAIL_MUSIC1 			buzzer_play_song(FAIL_MUSIC1, 120, 100)
#define	PLAY_FAIL_MUSIC2			buzzer_play_song(FAIL_MUSIC2, 120, 10)
#define	PLAY_OKAY_MUSIC1			buzzer_play_song(OKAY_MUSIC, 100, 0)
#define	PLAY_OKAY_MUSIC2			buzzer_play_song(OKAY_MUSIC2, 100, 0)
#define	CLICK_MUSIC						buzzer_play_song(CLICK, 20, 0)
#endif /* __BUZZER_SONG_H */
