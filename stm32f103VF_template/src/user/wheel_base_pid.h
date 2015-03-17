#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "wheel_base.h"
#include "special_char_handler.h"

#define time_interval 10
#define MIN(x,y) (x<y?x:y)
#define MAX(x,y) (x>y?x:y)
#define LIMITED(IN,L1,L2)  (L1<L2 ? (MAX(L1,MIN(IN,L2))) : (MAX(L2,MIN(IN,L1))))

#define xy_vel_limit	100
#define w_vel_limit		100

#define PID_XY_P	1500
#define PID_W_P		2000

static s32 pid_ratio_multiplier;
static s32 pid_ratio_divider;

typedef struct {
	s32 x;
	s32 y;
	s32 w;
} error;

typedef struct {
	error diff ;
	error derivative;
	error integral;
} PID_error;

typedef struct {
	s32 x;
	s32 y;
	s32 w;
} PID_output_vel;


void wheel_base_pid_update(void);

s32 get_vx(void);
s32 get_vy(void);
s32 get_vw(void);
s32 get_PID_err_diff_x (void);
s32 get_PID_err_diff_y (void);
s32 get_PID_err_diff_w (void);

void change_PID_flag (void);
void PID_init (void);
u8 get_pid_flag(void);
s8 get_target_changed_flag(void);
s8 get_change_count (void);




#endif /* __WHEEL_BASE_PID */

