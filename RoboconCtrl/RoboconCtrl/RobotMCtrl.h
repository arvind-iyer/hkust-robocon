#pragma once

#include <string>
#include <vector>

class RobotMCtrl
{
public:
	RobotMCtrl();
	std::string manual_mode(int x, int y, int w);
	std::string speed_mode(int speed);
	std::string coordinates(short x, short y, unsigned short angle);
	std::pair<std::vector<int>, BOOL> read(std::string string_received);
	std::string pid_toggle(BOOL pid);
	std::string special_keys(char key_to_send);
	std::string xbox_keys_part1(unsigned short xbox_digital, BYTE left_trigger, BYTE right_trigger, SHORT left_joy_x, SHORT left_joy_y);
	std::string xbox_keys_part2(SHORT right_joy_x, SHORT right_joy_y);
	~RobotMCtrl();
};

