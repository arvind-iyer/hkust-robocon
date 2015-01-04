// SerialTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "SerialIO.h"
#include <iostream>
#include <exception>
#include <windows.h>

int _tmain(int argc, _TCHAR* argv[])
{
	try {
		SerialIO a("COM5");
		a.write("Hello world!");
		Sleep(1000);
	}
	catch (std::runtime_error r) {
		std::cerr << r.what() << std::endl;
	}
	Sleep(1000);
	return 0;
}

