#pragma once

#include <windows.h>
#include <string>

class SerialIO
{
	HANDLE com_handle;
	bool connected;
	std::string port;
public:
	SerialIO(std::string port_name, int baud_rate = 115200);
	bool write(std::string msg);
	bool read();
	~SerialIO();
};

