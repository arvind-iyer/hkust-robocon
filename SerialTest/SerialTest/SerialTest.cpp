// SerialTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "SerialIO.h"
#include <iostream>
#include <string>
#include <exception>
#include <windows.h>
#include <process.h>
#include <conio.h>

bool running = true;

// Thread for reading from io port by polling using a while-loop

void read_thread(void* ioport){
	while (running){
		if (((SerialIO*)ioport)->bytes_to_read()) {
			std::cout << "Message received: " << ((SerialIO*)ioport)->read();
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

// Thread for realtime writing to io port by polling using a while-loop

void write_thread_realtime(void* ioport){
	while (running) {
		std::string msg;
		char ch;
		while ((ch = _getch()) != EOF) {
			if (!(msg.compare("q"))) {
				running = false;
			}
			else {
				std::cout << "Message sent: " << ch << std::endl;
				((SerialIO*)ioport)->write(std::string(1, ch));
			}
		}
	}
	_endthread();
}

int _tmain(int argc, _TCHAR* argv[])
{
	try {
		int baud_rate;
		int buffer_size;
		int mode;
		std::string port_name;

		//Configure options (perhaps I will add command line arguments later)
		std::cout << "Enter the number of the port you wish to open (number only please)." << std::endl;
		std::cin >> port_name;
		std::cout << "Enter the desired baud rate." << std::endl;
		std::cin >> baud_rate;
		std::cout << "Enter the size of the reading buffer." << std::endl;
		std::cin >> buffer_size;
		std::cout << "Choose mode of transmission: (0 for enter to send, 1 for realtime send)" << std::endl;
		std::cin >> mode;
		port_name = "COM" + port_name;
		std::cout << "Opening port: " << port_name << std::endl;

		SerialIO a(port_name, baud_rate, buffer_size);
		_beginthread(read_thread, 0, &a);
		if (mode){
			_beginthread(write_thread_realtime, 0, &a);
		}
		else {
			_beginthread(write_thread, 0, &a);
		}
		while (true){
			if (!running){
				int a;
				std::cout << "Quitting program." << std::endl;
				Sleep(1000);
				break;
			}
		}
	}
	catch (std::runtime_error r) {
		std::cerr << r.what() << std::endl;
		Sleep(2000);
	}
	return 0;
}

