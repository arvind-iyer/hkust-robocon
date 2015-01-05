#pragma once

#include <windows.h>
#include <string>
#include <iostream>

class SerialIO
{
	HANDLE com_handle;
	bool connected;
	std::string port;
public:
	SerialIO(std::string port_name, int baud_rate = 115200);
	bool write(std::string msg);
	int bytes_to_read();
	bool read(void *buffer, unsigned int limit);
	~SerialIO();
};

