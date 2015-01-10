/*
		Instruction: 
		1. complete manual control part
				a. void set_motor_rel()
				b. void manual_control()
				
		2. complete auto control
				
		Keywords to help you google related material:
		Omni-directional Drive, Holonomic Control, Vectors resolving, Kinematics
*/

#define threshold 20
#define motor_ratio 100
#define motor_limit (100 * motor_ratio * 50)
#define motor_max 	(100 * motor_ratio * 40)
#include "drive.h"
s32 x_img;	 	//X-coordinate image
s32 y_img;		//Y-coordinate image
s32 ang_img;	//Angle image
u8 circle_lock = 0, circle_lock_count = 0, circle_lock_case = 0;

struct PID {
	s32 Kp, Ki, Kd;
	s32 error, error_sum, error_change;
};

struct PID auto_x, auto_y, auto_w;

/**
  * @brief  Perform vector resolving and assign velocities to wheels,
						according to the desired direction and rotation, with respect to
						local co-ordinate system(the robot's frame)
  * @param  velocity-x, velocity-y, rotational velocity
  * @retval None
  */

// -1000 to 1000
s32 xbc_j(u8 joystick){
	s32 pos=0;
	pos = xbc_joy[joystick]*1000/30000;
	if (pos > -200 && pos < 200) pos = 0;
	if (pos > 1000) pos = 1000;
	if (pos < -1000) pos = -1000;
	return pos;
}

// ?_vel: -1000 to 1000
void set_motor_rel (s32 x_vel, s32 y_vel, s32 w_vel)
{
	u8 i;
	//variable for you to do calculation
	s32 vel[4] = {0,0,0,0};
	s32 cos45 = int_cos(45 * 10);
	u32 ratio = motor_max / 1000;
	
	circle_lock_case = 0;
	if (circle_lock) {
		if (test_ls(1,1,1,1)) {
			circle_lock_case = 1;
			// Go backward
			y_vel = -s_Abs(y_vel);
			y_vel -= s_Abs(x_vel);
		}				
		else if (test_ls(1, 0, 0, 0)) {
			circle_lock_case = 2;
			// Rotate anti-clockwise
			w_vel += s_Abs(x_vel);
		}
		else if (test_ls(0, 1, 0, 0)) {
			circle_lock_case = 3;
			// Rotate clockwise
			w_vel -= s_Abs(x_vel);
		}
		else if (test_ls(0, 0, 0, 0)) {
			circle_lock_case = 4;
			y_vel = Abs(y_vel);
			y_vel += s_Abs(x_vel);
			x_vel = 0;
		}
	}
	vel[0] = (-x_vel-y_vel);
	vel[1] = (y_vel-x_vel);
	vel[2] = (x_vel+y_vel);
	vel[3] = (x_vel-y_vel);

	for (i = 0; i < 4; i++) {
		vel[i] = (vel[i] * cos45 / 10000  + w_vel) * ratio;
		if (vel[i] > motor_limit) vel[i] = motor_limit;
		if (vel[i] < -motor_limit) vel[i] = -motor_limit;
	}
	

	for (i = 0; i < 4; i++) { 
		tft_prints(1, i, "#%d:", i);
		tft_prints(5, i, "[%ld]", vel[i]);
	}
	
	//tft_prints(1, 4, "get_X: %1d", get_X());
	//tft_prints(1, 5, "get_Y: %1d", get_Y());
	
	tft_prints(1, 4, "X: %1d %1d", xbc_j(XBC_LX), xbc_j(XBC_LY));
	tft_prints(1, 5, "Y: %1d %1d", xbc_j(XBC_RX), xbc_j(XBC_RY));
	
	tft_prints(1, 6, "CL: %1d", circle_lock);
	tft_prints(1, 7, "LS: %1d (%1d)", get_ls(), circle_lock_case);


	// set velocity to each motor
	lm629_velocity_start(FRONT_LEFT_MOTOR, vel[0]);
	lm629_velocity_start(FRONT_RIGHT_MOTOR, vel[1]);
	lm629_velocity_start(BACK_RIGHT_MOTOR, vel[2]);
	lm629_velocity_start(BACK_LEFT_MOTOR, vel[3]);
}

/**
  * @brief  Enter a user-controlled mode, receive signal from controller and move in the robot's frame.
						No need feedback and correction for this moment.
  * @param  None
  * @retval None
  */

void manual_control(void){
	while(1){
		if (ticks_img != get_ticks())
		{
			ticks_img = get_ticks();
			if(ticks_img % 50 == 0) // 1000/50 = 20Hz refresh rate
			{				
				tft_clear();
				xbc_update();					// button update
				x_img = get_X();			// x pos update
				y_img = get_Y();			// y pos update
				ang_img = get_angle();// angle pos update


				//TASK: use buttons to read the desired vector(direction + magnitude)
				button_update();
				
				set_motor_rel(xbc_j(XBC_LX), xbc_j(XBC_LY), -xbc_j(XBC_RX));
				
				tft_update();

				
				if (xbc_digital & XBC_Y) {
					circle_lock_count++;
					if (circle_lock_count == 5) {
						circle_lock = !circle_lock;
						if (circle_lock) buzzer_control(1, 15);
						else buzzer_control(3, 5);
					} else if (circle_lock_count > 5) {
						circle_lock_count = 5 + 1;
					}
				} else {
					circle_lock_count = 0;
				}

				
				if (button_down & WAKEUP) {
					set_motor_rel(0, 0, 0);
					return;
				}
				
			}
		}
	}
	
}


/**
  * @brief  Move to a specified point in global co-ordinate system
  * @param  destination x, y, angle in global co-ordinate system
  * @retval None
  */


u8 auto_control(s32 target_x, s32 target_y, s32 target_w){

	u8 arrived = 0;
	s32 x_vel = 0, y_vel = 0, w_vel = 0;
	s32 dx = target_x - x_img, dy = target_y - y_img, dw = target_w - ang_img;
	
	if (dw >= 1800) {
		dw -= 3600;
	}	else if (dw < -1800) {
		dw += 3600;
	}
	
	
	// X
	auto_x.error_change = dx - auto_x.error;
	auto_x.error = dx;
	auto_x.error_sum += dx;
	if (Abs(dx) > threshold) x_vel = auto_x.error * auto_x.Kp / 100 + auto_x.error_sum * auto_x.Ki / 100 + auto_x.error_change * auto_x.Kd / 100;

	
	// Y
	auto_y.error_change = dy - auto_y.error;
	auto_y.error = dy;
	auto_y.error_sum += dy;
	if (Abs(dy) > threshold) y_vel = auto_y.error * auto_y.Kp / 100 + auto_y.error_sum * auto_y.Ki / 100 + auto_y.error_change * auto_y.Kd / 100;

	// W
	auto_w.error_change = dw - auto_w.error;
	auto_w.error = dw;
	auto_w.error_sum += dw;
	if (Abs(dw) > threshold) w_vel = auto_w.error * auto_w.Kp / 100 + auto_w.error_sum * auto_w.Ki / 100;


	tft_prints(1, 9, "d: %1d, %1d, %1d", dx, dy, dw);

	xy_rotate(&x_vel, &y_vel, ang_img);

	set_motor_rel(x_vel, y_vel, -w_vel);
	
	return (Abs(dx) <= threshold && Abs(dy) <= threshold && Abs(dw) <= threshold);
	
}

void auto_init(s32 target_x, s32 target_y, s32 target_w) {
	// Init x
	auto_x.error = target_x - x_img;
	auto_x.error_change = 0;
	auto_x.error_sum = 0;
	auto_x.Kp = 5000;
	auto_x.Ki = 10;
	auto_x.Kd = 10;
	
	// Init y
	auto_y.error = target_y - y_img;
	auto_y.error_change = 0;
	auto_y.error_sum = 0;
	auto_y.Kp = 5000;
	auto_y.Ki = 10;
	auto_y.Kd = 10;
	
	// Init w
	auto_w.error = target_w - ang_img;
	auto_w.error_change = 0;
	auto_w.error_sum = 0;
	auto_w.Kp = 300;
	auto_w.Ki = 1;
	auto_w.Kd = 1;
}
	
void auto_test() {
	u8 action = 0, arrived;
	s32 target_x, target_y, target_angle;

	while(1){
		if (ticks_img != get_ticks())
		{
			ticks_img = get_ticks();
			if(ticks_img % 50 == 0) // 1000/50 = 20Hz refresh rate
			{
				xbc_update();						// button update
				x_img = get_X();				// x pos update
				y_img = get_Y();				// y pos update
				ang_img = get_angle();	// angle pos update
				
				
				
				//TASK: use buttons to read the desired vector(direction + magnitude)
				button_update();
				
				tft_clear();
				tft_prints(1, 8, "Auto: %1d", action);
				
				if (button_data & DOWN) {
					action = 1;
					target_x = 500;
					target_y = -300;
					target_angle = 900;
					auto_init(target_x, target_y, target_angle);
					
				} else if (button_data & RIGHT) {
					target_x = 0;
					target_y = 0;
					target_angle = 450;
					action = 1;
					auto_init(target_x, target_y, target_angle);
				} else if (button_data & UP) {
					action = 2;
					target_x = 0;
					target_y = 0;
					target_angle = 0;
					auto_init(target_x, target_y, target_angle);
				} else if (button_data & LEFT) {
					target_x = 0;
					target_y = 0;
					target_angle = 2700;
					action = 1;
					auto_init(target_x, target_y, target_angle);
				} else if (button_data & START) {
					action = 0;
				} else {
					
					if (action > 0) {
						arrived = auto_control(target_x, target_y, target_angle);
						if (arrived) {
							action = 0;
						}
					} else {
						set_motor_rel(0, 0, 0);
					}
				}
				
				if (xbc_digital & XBC_B) {
					// Emergency stop
					action = 0;
				}

				if (button_down & WAKEUP) {
					action = 0;
					set_motor_rel(0, 0, 0);
					return;
				}
				
				tft_update();
			}
		}
	}
}
