#pragma once

#include <string>
#include <vector>

class RobotMCtrl
{
public:
	RobotMCtrl();
	std::string operator()(int x, int y, int w);
	std::string operator()(int speed);
	std::string operator()(int x, int y, unsigned int theta);
	std::pair<std::vector<int>, BOOL> operator()(std::string string_received);
	~RobotMCtrl();
};

