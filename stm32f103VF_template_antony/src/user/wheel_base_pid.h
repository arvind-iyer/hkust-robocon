#ifndef	__WHEEL_BASE_PID
#define	__WHEEL_BASE_PID

#include "stm32f10x.h"
#include "gyro.h"
#include "wheel_base.h"
#include "special_char_handler.h"

#define CEILDIV(x,y) ((x)+((y)/2)/(y))

#define SIGN(x) (x<0?-1:(x==0?0:1))
#define ABS(x) (x>0?x:-x)
#define MIN(x,y) (x<y?x:y)
#define MAX(x,y) (x>y?x:y)
#define LIMITED(IN,L1,L2)  (L1<L2 ? (MAX(L1,MIN(IN,L2))) : (MAX(L2,MIN(IN,L1))))

#define ANGLE_PN1800(x) ((x%3600)>1800 ? ((x%3600)-3600) : (x%3600))
#define ANGLE_0_3600(x) (x%3600)

#define DIR_X	0
#define DIR_Y	0	// originally 1
#define ANGLE	2

#define VELOCITY_LIMIT_X 100
#define VELOCITY_LIMIT_Y 100
#define VELOCITY_LIMIT_W 0

#define POSITION_FIXING_SPEED 4
#define POSITION_FIXING_THRESHOLD 10

//                  X       Y       W 		[4 decimal places]
static s32
	$kp[3] = {  1500,      0,      0},
	$ki[3] = {     0,      0,      0},
	$kd[3] = {     0,      0,      0};
#define K_DIVIDER 10000							//		[4 decmial places]

static const u16 PID_SPEED_MODE[10] =	// In percentage (20 = 20%)
{
	0, 10, 20, 30, 40, 50, 60, 70, 80, 90
};
static POSITION $current, $target;
static POSITION $error, $last_error;
static POSITION $in, $de;

static WHEEL_BASE_VEL $set_vel;
static WHEEL_BASE_VEL $unlimit_vel;
static u8 $speed_mode;
static WHEEL_BASE_VEL $vel_limit;
	
void wheel_base_pid_init(void);
void wheel_base_pid_update(void);
WHEEL_BASE_VEL wheel_base_pid_current_vel(void);
WHEEL_BASE_VEL wheel_base_pid_unlimit_vel(void);
s32 get_kpx(void);
s32 get_kix(void);
s32 get_kdx(void);
s32 get_kpa(void);
u8 get_speed_mode(void);
s16 get_angle_error(void);
WHEEL_BASE_VEL get_vel_limit(void);

#endif /* __WHEEL_BASE_PID */

