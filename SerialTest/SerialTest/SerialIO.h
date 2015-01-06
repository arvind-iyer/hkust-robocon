#pragma once

#include <windows.h>
#include <string>
#include <iostream>

class SerialIO
{
	HANDLE com_handle;
	bool connected;
	std::string port;
	char* buffer;
	int buffer_size;
public:
	SerialIO(std::string port_name, int baud_rate = 115200, int size_of_buffer = 500);
	bool write(std::string msg);
	int bytes_to_read();
	std::string read();
	~SerialIO();
};

