#ifndef __AUTO_TIMER_H
#define __AUTO_TIMER_H

typedef struct {
	void (*start)(void);
	void (*update)(void);
} auto_timer;

// robot timer structure
extern auto_timer robot_timer;

#endif
