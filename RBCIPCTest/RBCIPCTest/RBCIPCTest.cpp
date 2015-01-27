// RBCIPCTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "windows.h"
#include <iostream>

struct coord_data {
	int x_coord;
	int y_coord;
	unsigned int angle;
};

int _tmain(int argc, _TCHAR* argv[])
{
	HWND wnd_handle = FindWindowEx(NULL, NULL, NULL, _T("Main Control Panel - RoboconCtrl"));
	if (wnd_handle != NULL) {
		std::cout << "We're in business!" << std::endl;
		bool run = true;
		while (run) {
			int x, y, angle;
			std::cout << "Input x coord:" << std::endl;
			std::cin >> x;
			std::cout << "Input y coord:" << std::endl;
			std::cin >> y;
			std::cout << "Input angle: " << std::endl;
			std::cin >> angle;
			coord_data d = { x, y, angle };
			COPYDATASTRUCT c = { 0x06050116, sizeof(d), &d };
			SendMessage(wnd_handle, WM_COPYDATA, (WPARAM)GetConsoleWindow(), (LPARAM)&c);
			std::cout << "Message sent! Input 1 to send again" << std::endl;
			int a;
			std::cin >> a;
			if (a != 1) {
				run = false;
			}
		}
	}
	else {
		std::cout << "Open the program, idiot." << std::endl;
		int a;
		std::cin >> a;
	}
	return 0;
}

