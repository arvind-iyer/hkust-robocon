#include "stdafx.h"
#include "RobotMCtrl.h"
#include <cmath>
#include <sstream>
#include <iomanip>

RobotMCtrl::RobotMCtrl()
{
}

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

std::string RobotMCtrl::manual_mode(int x, int y, int w) {
	// Bounds checking
	if (x > 100) {
		x = 100;
	}
	else if (x < -100) {
		x = -100;
	}
	if (y > 100) {
		y = 100;
	}
	else if (y < -100) {
		y = -100;
	}
	if (w > 100) {
		w = 100;
	}
	else if (w < -100) {
		w = -100;
	}

	if (x != 0 && y != 0) {
		double scale = sqrt(100 * 100 + 100 * 100) / 100.0;
		x = (int)((double)x / scale);
		y = (int)((double)y / scale);
	}

	BYTE soh = 0x12;
	BYTE id = 0x40;
	BYTE l = 3;
	char coordinates[3] = { (char)x, (char)y, char(w) };
	BYTE eot = 0x34;
	char buffer[2] = { 0, 0 };
	crc16(buffer, coordinates, 3);
	std::ostringstream o;
	o << soh << id << l << coordinates[0] << coordinates[1] << coordinates[2] << id << buffer[0] << buffer[1] << eot;

	return o.str();
}

std::string RobotMCtrl::speed_mode(int speed) {
	if (speed < 0) {
		speed = 0;
	}
	else if (speed > 9) {
		speed = 9;
	}

	BYTE soh = 0x12;
	BYTE id = 0x41;
	BYTE l = 1;
	char data[1] = { (char)speed };
	BYTE eot = 0x34;
	char buffer[2] = { 0, 0 };
	crc16(buffer, data, 1);
	std::ostringstream o;
	o << soh << id << l << data[0] << id << buffer[0] << buffer[1] << eot;

	return o.str();
	
}

std::pair<std::vector<int>, BOOL> RobotMCtrl::read(std::string string_received) {
	// Determines whether or not both a start and a stop byte are present in the string
	if (string_received.find_first_of(0x12) != std::string::npos && string_received.find_first_of(0x34) != std::string::npos) {
		// Prune the start and stop bits from the string
		std::string string = string_received.substr(string_received.find_first_of((char)0x12) + 1 , string_received.find_last_of((char)0x34) - string_received.find_first_of((char)0x12) - 1);
		// Checks if data size is correct, and ID is correct
		if (string.size() == 11 && string[0] == 0x60 && string[1] == 6 && string[8] == 0x60) {
			char data[6];
			for (int i = 0; i < 6; i++) {
				data[i] = string[i + 2];
			}
			char buffer[2] = { 0, 0 };
			crc16(buffer, data, 6);
			// Verifies data using CRC
			if (string[9] == buffer[0] && string[10] == buffer[1]) {
				
				std::vector<int> coordinates;

				// When reconstructing the bytes, casting to unsigned is required to prevent sign extension
				short x = ((BYTE)string[2] << 8) | ((BYTE)string[3]);
				coordinates.push_back(x);
				short y = ((BYTE)string[4] << 8) | ((BYTE)string[5]);
				coordinates.push_back(y);

				// angle is unsigned to ensure the sign is not messed up
				// don't you love automatic sign extension?
				unsigned short angle = ((BYTE)string[6] << 8) | ((BYTE)string[7]);
				coordinates.push_back(angle);

				return std::make_pair(coordinates, TRUE);
			}
		}
	}
	return std::make_pair(std::vector<int>(), FALSE);
}

std::string RobotMCtrl::coordinates(short x, short y, unsigned short angle)
{
	char soh = 0x12;
	char id = 0x50;
	char data_length = 0x06;
	char data[6] = { (BYTE)(x >> 8), (BYTE)(x), (BYTE)(y >> 8), (BYTE)(y), (BYTE)(angle >> 8), (BYTE)(angle) };
	char buffer[2] = { 0, 0 };
	char eot = 0x34;
	crc16(buffer, data, 6);

	// construct the string to send
	std::stringstream o;
	o << soh << id << data_length;
	for (int i = 0; i < 6; ++i) {
		o << data[i];
	}
	o << id << buffer[0] << buffer[1] << eot;

	// Casting is needed to suppress warnings

	BYTE x1 = (BYTE)(x >> 8);
	BYTE x2 = (BYTE)(x);

	BYTE y1 = (BYTE)(y >> 8);
	BYTE y2 = (BYTE)(y);

	BYTE angle1 = (BYTE)(angle >> 8);
	BYTE angle2 = (BYTE)(angle);

	std::basic_ostringstream<TCHAR> oss;
	oss << std::hex << std::setfill(_T('0')) << std::setw(2) << (BYTE)x1 << _T(" ") << std::setw(2) << (BYTE)x2 << _T(" ") << std::setw(2) << (BYTE)y1 << _T(" ") << std::setw(2) << (BYTE)y2 << _T(" ") << std::setw(2) << (BYTE)angle1 << _T(" ") << std::setw(2) << (BYTE)angle2 << _T(" ");
	oss << std::endl;
	OutputDebugString(oss.str().c_str());

	std::basic_ostringstream<TCHAR> oss2;
	oss2 << std::hex << std::setfill(_T('0')) << std::setw(4) << x << _T(" ") << std::setw(4) << y << _T(" ") << std::setw(4) << angle;
	oss2 << std::endl;
	OutputDebugString(oss2.str().c_str());

	return o.str();
}

std::string RobotMCtrl::pid_toggle(BOOL pid)
{
	if (pid) {
		char soh = 0x12;
		char id = 0x51;
		char data_length = 0x00;
		char data = 0;
		char buffer[2] = { 0, 0 };
		char eot = 0x34;

		crc16(buffer, &data, 0);

		// construct the string to send
		std::stringstream o;
		o << soh << id << data_length << id << buffer[0] << buffer[1] << eot;
		return o.str();
	}
	else {

		char soh = 0x12;
		char id = 0x52;
		char data_length = 0x00;
		char* data = 0;
		char buffer[2] = { 0, 0 };
		char eot = 0x34;

		crc16(buffer, data, 0);

		// construct the string to send
		std::stringstream o;
		o << soh << id << data_length << id << buffer[0] << buffer[1] << eot;
		return o.str();
	}
}

std::string RobotMCtrl::special_keys(char key_to_send)
{
	char soh = 0x12;
	char id = 0x70;
	char data_length = 0x01;
	char data[1] = { key_to_send };
	char buffer[2] = { 0, 0 };
	char eot = 0x34;

	crc16(buffer, data, 1);

	// construct string to send
	std::stringstream o;
	o << soh << id << data_length << data[0] << id << buffer[0] << buffer[1] << eot;
	return o.str();
}

std::string RobotMCtrl::xbox_keys_part1(unsigned short xbox_digital, BYTE left_trigger, BYTE right_trigger, SHORT left_joy_x, SHORT left_joy_y)
{
	char soh = 0x12;
	unsigned char id = 0x90;
	char data_length = 0x08;
	char data[8] = { (BYTE)(xbox_digital), (BYTE)(xbox_digital >> 8), left_trigger, right_trigger, (BYTE)(left_joy_x), (BYTE)(left_joy_x >> 8), (BYTE)(left_joy_y), (BYTE)(left_joy_y >> 8) };
	char buffer[2] = { 0, 0 };
	char eot = 0x34;
	crc16(buffer, data, 8);

	// construct string to send
	std::stringstream o;
	o << soh << static_cast<char>(id) << data_length;

	for (int i = 0; i < 8; ++i)
	{
		o << data[i];
	}
	o << static_cast<char>(id) << buffer[0] << buffer[1] << eot;
	return o.str();
}

std::string RobotMCtrl::xbox_keys_part2(SHORT right_joy_x, SHORT right_joy_y)
{
	char soh = 0x12;
	unsigned char id = 0x91;
	char data_length = 0x04;
	char data[4] = { (BYTE)(right_joy_x), (BYTE)(right_joy_x >> 8), (BYTE)(right_joy_y), BYTE(right_joy_y >> 8) };
	char buffer[2] = { 0, 0 };
	char eot = 0x34;
	crc16(buffer, data, 4);
	// construct string to send
	std::stringstream o;
	o << soh << static_cast<char>(id) << data_length;

	for (int i = 0; i < 4; ++i)
	{
		o << data[i];
	}

	o << static_cast<char>(id) << buffer[0] << buffer[1] << eot;
	return o.str();
}

RobotMCtrl::~RobotMCtrl()
{
}
