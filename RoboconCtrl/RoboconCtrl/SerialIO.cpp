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
/*
bool fletcher16(char* buffer, const char* message, size_t msg_length) {
	
//	unsigned int sum1 = 0, sum2 = 0;
//	for (int i = 0; i < msg_length; ++i) {
//		sum1 = (sum1 + (unsigned int)(message[i])) % 255;
//		sum2 = (sum2 + sum1) % 255;
//	}
//	buffer[0] = (unsigned char)(sum2);
//	buffer[1] = (unsigned char)(sum1);
	
	
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
	
	//Second reduction step to reduce sums to 8 bits
	sum1 = (sum1 & 0xff) + (sum1 >> 8);
	sum2 = (sum2 & 0xff) + (sum2 >> 8);

	buffer[0] = (unsigned char)sum2;
	buffer[1] = (unsigned char)sum1;
	
	return true;
}
*/

static const unsigned short crc16table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

bool crc16(char* buffer, const char* message, int msg_l)
{
	// taken from http://www.menie.org/georges/embedded/crc16.html
	int counter;
	unsigned short crc = 0;
	for (counter = 0; counter < msg_l; counter++) {
		crc = (crc << 8) ^ crc16table[((crc >> 8) ^ *(char *)message++) & 0x00FF];
	}
	buffer[0] = (unsigned char)(crc >> 8);
	buffer[1] = (unsigned char)crc;
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
		crc16(checkbytes, msg.c_str(), msg.size());
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
