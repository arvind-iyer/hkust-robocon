#pragma once

#include <string>

class RobotMCtrl
{
public:
	RobotMCtrl();
	std::string operator()(int x, int y, int w);
	std::string operator()(int speed);
	~RobotMCtrl();
};

