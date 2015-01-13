#include "stdafx.h"
#include "SerialIO.h"
#include <locale>
#include <codecvt>
#include <fstream>
#include <sstream>
#include <exception>

std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

SerialIO::SerialIO(std::basic_string<TCHAR> port_name, int baud_rate, unsigned size_of_buffer) : connected(false), buffer(new char[size_of_buffer]), buffer_size(size_of_buffer), port(_T("\\\\.\\") + port_name), com_handle(CreateFile((_T("\\\\.\\") + port_name).c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL))
{	
	try {
		/*
		*Check if handle is valid
		*/
		if (com_handle == INVALID_HANDLE_VALUE)
		{
			//If not success full display an error
			if (GetLastError() == ERROR_FILE_NOT_FOUND){
				//Print specific message for invalid port
				std::ostringstream error_stream;
				error_stream << "ERROR: Handle was not attached, " << converter.to_bytes(port_name).c_str() << " port not available.";
				throw std::runtime_error(error_stream.str());
			}
			else
			{
				throw std::runtime_error("ERROR: Reason unknown.");
			}
		}
		else {
			std::cout << "Connected!" << std::endl;
			connected = true;

			//If connected we try to set the comm parameters using DCB
			DCB dcbSerialParams = { 0 };
			COMMTIMEOUTS timeouts = { 0 };

			//Try to get the current
			if (!GetCommState(com_handle, &dcbSerialParams))
			{
				//If impossible, show an error
				throw std::runtime_error("ERROR: Failed to get current serial parameters.");
			}
			else
			{
				//Define serial connection parameters
				dcbSerialParams.BaudRate = baud_rate;
				dcbSerialParams.ByteSize = 8;
				dcbSerialParams.StopBits = ONESTOPBIT;
				dcbSerialParams.Parity = NOPARITY;

				//Set the parameters and check for their proper application
				if (!SetCommState(com_handle, &dcbSerialParams))
				{
					throw std::runtime_error("ERROR: Could not set port parameters.");
				}
				else
				{
					// Set COM port timeout settings
					timeouts.ReadIntervalTimeout = 50;
					timeouts.ReadTotalTimeoutConstant = 50;
					timeouts.ReadTotalTimeoutMultiplier = 10;
					timeouts.WriteTotalTimeoutConstant = 50;
					timeouts.WriteTotalTimeoutMultiplier = 10;
					if (!SetCommTimeouts(com_handle, &timeouts))
					{
						throw std::runtime_error("ERROR: Cannot set timeouts.");
					}
					std::cout << "Port configured!" << std::endl;
				}
			}
		}
	}
	catch (std::runtime_error r) {
		if (buffer) {
			delete[] buffer;
			buffer = NULL;
		}
		throw r;
	}
}

bool SerialIO::_internal_write(std::string string_to_write)
{
	OVERLAPPED osWrite = { 0 };
	DWORD dwWritten;
	bool fRes;

	// Create this writes OVERLAPPED structure hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		// Error creating overlapped event handle.
		return FALSE;
	}

	// Issue write.
	if (!WriteFile(com_handle, string_to_write.c_str(), string_to_write.size(), &dwWritten, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING) {
			// WriteFile failed, but it isn't delayed. Report error and abort.
			fRes = FALSE;
		}
		else {
			// Write is pending.
			if (!GetOverlappedResult(com_handle, &osWrite, &dwWritten, TRUE))
				fRes = FALSE;
			else
				// Write operation completed successfully.
				fRes = TRUE;
		}
	}
	else
	{
		fRes = TRUE;
	}
	CloseHandle(osWrite.hEvent);
	return fRes;
}

bool fletcher16(char* buffer, const char* message, size_t msg_length) {
	/*
	unsigned int sum1 = 0, sum2 = 0;
	for (int i = 0; i < msg_length; ++i) {
		sum1 = (sum1 + (unsigned int)(message[i])) % 255;
		sum2 = (sum2 + sum1) % 255;
	}
	buffer[0] = (unsigned char)(sum1);
	buffer[1] = (unsigned char)(sum2);
	*/
	
	// This code is adapted from wikipedia

	UINT16 sum1 = 0xff, sum2 = 0xff;

	while (msg_length) {
		size_t tlen = msg_length > 20 ? 20 : msg_length;
		msg_length -= tlen;
		do {
			sum2 += sum1 += *(BYTE*)(message++);
		} while (--tlen);
		sum1 = (sum1 & 0xff) + (sum1 >> 8);
		sum2 = (sum2 & 0xff) + (sum2 >> 8);
	}
	
	/* Second reduction step to reduce sums to 8 bits */
	
	sum1 = (sum1 & 0xff) + (sum1 >> 8);
	sum2 = (sum2 & 0xff) + (sum2 >> 8);

	buffer[0] = (unsigned char)sum1;
	buffer[1] = (unsigned char)sum2;
	
	return true;
}

bool SerialIO::write_string_with_padbytes(std::string msg)
{
	if ((unsigned)msg.size() > UCHAR_MAX) {
		for (unsigned int i = 0; i < msg.length(); i += UCHAR_MAX) {
			write_string_with_padbytes(msg.substr(i, UCHAR_MAX));
		}
		return true;
	}
	else {
		// byte padding
		// add wake byte, check bits, and sleep byte
		char soh = std::stoi("01", 0, 16);
		unsigned char datalength = msg.size();
		char eot = std::stoi("04", 0, 16);
		//char checkbytes1 = 0, checkbytes2 = 0;
		char checkbytes[2] = { 0, 0 };
		fletcher16(checkbytes, msg.c_str(), msg.size());
		std::basic_ostringstream<TCHAR> oss;
		oss << _T("SOH BYTE: ") << std::stoi("01", 0, 16) << _T(" EOT BYTE: ") << std::stoi("04", 0, 16);
		OutputDebugString(oss.str().c_str());

		std::ostringstream o;
		o << soh << (char)datalength << msg << checkbytes[0] << checkbytes[1] << eot << eot;
		msg = o.str();
		return _internal_write(msg);
	}
}

bool SerialIO::write(std::basic_string<TCHAR> msg_string, int padbytes)
{
	std::string msg(converter.to_bytes(msg_string));
	if (padbytes) {
		return write_string_with_padbytes(msg);
	}
	else {
		return _internal_write(msg);
	}
}

int SerialIO::bytes_to_read()
{
	DWORD dwErrorFlags;
	COMSTAT ComStat;

	ClearCommError(com_handle, &dwErrorFlags, &ComStat);
	return((int)ComStat.cbInQue);
}

bool SerialIO::is_connected()
{
	return connected;
}

std::string SerialIO::read()
{
	BOOL bReadStatus;
	DWORD dwBytesRead, dwErrorFlags;
	COMSTAT ComStat;
	OVERLAPPED osRead = {0};
	memset(buffer, 0, sizeof(char)*buffer_size);

	ClearCommError(com_handle, &dwErrorFlags, &ComStat);
	if (!ComStat.cbInQue) {
		return std::string();
	}

	dwBytesRead = (DWORD)ComStat.cbInQue;
	if (buffer_size < dwBytesRead) {
		dwBytesRead = buffer_size;
		std::basic_ostringstream<TCHAR> oss;
		oss << dwBytesRead;
		OutputDebugString(oss.str().c_str());
	}

	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL) {
		return std::string();
	}

	bReadStatus = ReadFile(com_handle, buffer, dwBytesRead, &dwBytesRead, &osRead);
	if (!bReadStatus){
		if (GetLastError() == ERROR_IO_PENDING){
			WaitForSingleObject(osRead.hEvent, 2000);
			return std::string();
		}
		return std::string();
	}
	CloseHandle(osRead.hEvent);
	if (buffer[500] != '\0') {
		OutputDebugString(_T("Oh"));
	}
	return std::string(buffer, dwBytesRead);
}

SerialIO::~SerialIO()
{
	if (connected) {
		std::cout << "Closing port" << std::endl;
		CloseHandle(com_handle);
	}
	if (buffer) {
		delete[] buffer;
	}
}
