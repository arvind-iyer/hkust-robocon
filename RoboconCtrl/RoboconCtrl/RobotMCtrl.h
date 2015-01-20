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
	~RobotMCtrl();
};

