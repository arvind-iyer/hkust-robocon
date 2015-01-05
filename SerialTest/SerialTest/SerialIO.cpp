#include "stdafx.h"
#include "SerialIO.h"
#include <locale>
#include <codecvt>
#include <fstream>
#include <sstream>
#include <exception>

std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

SerialIO::SerialIO(std::string port_name, int baud_rate) : port("\\\\.\\" + port_name), connected(false), com_handle(CreateFile((converter.from_bytes("\\\\.\\" + port_name)).c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL))
{
	// std::wcout << (converter.from_bytes("\\\\.\\" + port_name)).c_str() << std::endl;
	
	/*
	 *Check if handle is valid
	 */
	if (com_handle == INVALID_HANDLE_VALUE)
	{
		//If not success full display an error
		if (GetLastError() == ERROR_FILE_NOT_FOUND){
			//Print specific message for invalid port
			std::ostringstream error_stream;
			error_stream << "ERROR: Handle was not attached, " << port_name.c_str() << " port not available.";
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

bool SerialIO::write(std::string msg)
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
	if (!WriteFile(com_handle, msg.c_str(), msg.size(), &dwWritten, &osWrite)) {
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

int SerialIO::bytes_to_read()
{
	DWORD dwErrorFlags;
	COMSTAT ComStat;

	ClearCommError(com_handle, &dwErrorFlags, &ComStat);
	return((int)ComStat.cbInQue);
}

bool SerialIO::read(void *buffer, unsigned int limit)
{
	BOOL bReadStatus;
	DWORD dwBytesRead, dwErrorFlags;
	COMSTAT ComStat;
	OVERLAPPED osRead = {0};

	ClearCommError(com_handle, &dwErrorFlags, &ComStat);
	if (!ComStat.cbInQue) {
		return(0);
	}

	dwBytesRead = (DWORD)ComStat.cbInQue;
	if (limit < dwBytesRead) {
		dwBytesRead = limit;
	}

	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL) {
		return FALSE;
	}

	bReadStatus = ReadFile(com_handle, buffer, dwBytesRead, &dwBytesRead, &osRead);
	if (!bReadStatus){
		if (GetLastError() == ERROR_IO_PENDING){
			WaitForSingleObject(osRead.hEvent, 2000);
			return((int)dwBytesRead);
		}
		return(0);
	}
	CloseHandle(osRead.hEvent);
	return((int)dwBytesRead);
}

SerialIO::~SerialIO()
{
	if (connected) {
		std::cout << "Closing port" << std::endl;
		CloseHandle(com_handle);
	}
}
