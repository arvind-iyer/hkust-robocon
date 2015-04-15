#pragma once

#include <windows.h>
#include <string>
#include <iostream>

class SerialIO
{
private:
	HANDLE com_handle;
	bool connected;
	std::basic_string<TCHAR> port;
	char* buffer;
	unsigned buffer_size;
	bool _internal_write(std::string string_to_write);
//	bool write_string_with_padbytes(std::string msg);

public:
	SerialIO(std::basic_string<TCHAR> port_name, int baud_rate = 115200, unsigned size_of_buffer = 500);
	bool write(std::basic_string<TCHAR> msg);
	bool write(std::string msg);
	int bytes_to_read();
	bool is_connected();
	std::string read();
	~SerialIO();
};

