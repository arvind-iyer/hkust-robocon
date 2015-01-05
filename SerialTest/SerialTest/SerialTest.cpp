// SerialTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "SerialIO.h"
#include <iostream>
#include <string>
#include <exception>
#include <windows.h>
#include <process.h>

bool running = true;

// Thread for reading from io port by polling using a while-loop

void read_thread(void* ioport){
	while (running){
		if (((SerialIO*)ioport)->bytes_to_read()) {
			char abc[500] = { 0 };
			((SerialIO*)ioport)->read(&abc, 500);
			std::cout << "Message received: " << abc;
		}
	}
	_endthread();
}

// Thread for writing to io port by polling using a while-loop

void write_thread(void* ioport){
	while (running) {
		std::string msg;
		std::cin >> msg;
		if (!(msg.compare("q"))) {
			running = false;
		}
		else {
			std::cout << "Message sent: " << msg << std::endl;
			((SerialIO*)ioport)->write(msg);
		}
	}
	_endthread();
}

int _tmain(int argc, _TCHAR* argv[])
{
	try {
		SerialIO a("COM5");
		_beginthread(read_thread, 0, &a);
		_beginthread(write_thread, 0, &a);
		while (true){
			if (!running){
				Sleep(500);
				break;
			}
		}
	}
	catch (std::runtime_error r) {
		std::cerr << r.what() << std::endl;
	}
	return 0;
}

