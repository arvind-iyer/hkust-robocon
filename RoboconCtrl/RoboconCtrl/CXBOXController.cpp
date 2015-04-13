#include "stdafx.h"
#include "CXBOXController.h"
#include "SerialIO.h"
#include "MainFrm.h"
#include "RobotMCtrl.h"

#include <sstream>
#include <iomanip>
#include <XInput.h>
#define XBC_UP		0x0001
#define XBC_DOWN	0x0002
#define XBC_LEFT	0x0004
#define XBC_RIGHT	0x0008
#define XBC_START	0x0010
#define XBC_BACK	0x0020
#define XBC_L_JOY	0x0080
#define XBC_R_JOY	0x0040
#define XBC_LB		0x0100
#define XBC_RB		0x0200
#define XBC_XBOX	0x0400
#define XBC_A		0x1000
#define XBC_B		0x2000
#define XBC_X		0x4000
#define XBC_Y		0x8000

namespace {
	bool running = true;
}

// Now, the XInput Library
// NOTE: COMMENT THIS OUT IF YOU ARE NOT USING A COMPILER THAT SUPPORTS THIS METHOD OF LINKING LIBRARIES

#pragma comment(lib, "XINPUT9_1_0.lib")

namespace {
	// XBOX Controller Class Definition
	class CXBOXController
	{
	private:
		XINPUT_STATE _controller_state;
		int _controller_num;
	public:
		CXBOXController(int player_number) : _controller_num(player_number - 1) {};
		XINPUT_STATE GetState() {
				// Zeroise the state
				ZeroMemory(&_controller_state, sizeof(XINPUT_STATE));

				// Get the state
				XInputGetState(_controller_num, &_controller_state);
				return _controller_state;
		};
		bool is_connected() {
			// Zeroise the state
			ZeroMemory(&_controller_state, sizeof(XINPUT_STATE));

			// Get the state
			DWORD result = XInputGetState(_controller_num, &_controller_state);

			return result == ERROR_SUCCESS ? true : false;
		};
		void vibrate(int left_val = 0, int right_val = 0) {
			// Create a Vibraton State
			XINPUT_VIBRATION vibration;

			// Zeroise the Vibration
			ZeroMemory(&vibration, sizeof(XINPUT_VIBRATION));

			// Set the Vibration Values
			vibration.wLeftMotorSpeed = left_val;
			vibration.wRightMotorSpeed = right_val;

			// Vibrate the controller
			XInputSetState(_controller_num, &vibration);
		};
	};
}

void terminate_thread()
{
	running = false;
}

#define DEAD_ZONE 6000

UINT __cdecl xbox_write_thread(LPVOID app_ptr)
{
	SerialIO** serial = static_cast<SerialIO**>(app_ptr);
	CXBOXController* controller = nullptr;
	// wait for xbox to be connected
	while (running) {
		while (controller == nullptr && running)
		{
			if (AfxGetApp() && ((CMainFrame*)AfxGetApp()->GetMainWnd()) && (((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())) {
				(((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())->PostMessage(UWM_RECEIVE_XBOX, 0, 0);
			}
			controller = new CXBOXController(1);
			if (!controller->is_connected()) {
				delete controller;
				controller = nullptr;
			}
			else {
				break;
			}
			controller = new CXBOXController(2);
			if (!controller->is_connected()) {
				delete controller;
				controller = nullptr;
			}
			else {
				break;
			}
			controller = new CXBOXController(3);
			if (!controller->is_connected()) {
				delete controller;
				controller = nullptr;
			}
			else {
				break;
			}
			controller = new CXBOXController(4);
			if (!controller->is_connected()) {
				delete controller;
				controller = nullptr;
			}
			else {
				break;
			}
			Sleep(50);
		}

		while (controller && controller->is_connected() && running)
		{
			if (AfxGetApp() && ((CMainFrame*)AfxGetApp()->GetMainWnd()) && (((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())) {
				(((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())->PostMessage(UWM_RECEIVE_XBOX, 0, 1);
			}
			if (serial != nullptr && *serial != nullptr && running) {

				XINPUT_GAMEPAD x = controller->GetState().Gamepad;

				short lx = x.sThumbLX, ly = x.sThumbLY, rx = x.sThumbRX, ry = x.sThumbRY;
				BYTE lt = x.bLeftTrigger, rt = x.bRightTrigger;

				// scale to 32767

				// dead zone correction

				// lx
				if (lx < DEAD_ZONE && lx > -DEAD_ZONE) {
					lx = 0;
				}
				else if (lx >= DEAD_ZONE) {
					lx = (lx - DEAD_ZONE) * 32767 / (32767 - DEAD_ZONE);
				}
				else {
					lx = (lx + DEAD_ZONE) * 32768 / (32768 - DEAD_ZONE);
				}

				// ly
				if (ly < DEAD_ZONE && ly > -DEAD_ZONE) {
					ly = 0;
				}
				else if (ly >= DEAD_ZONE) {
					ly = (ly - DEAD_ZONE) * 32767 / (32767 - DEAD_ZONE);
				}
				else {
					ly = (ly + DEAD_ZONE) * 32768 / (32768 - DEAD_ZONE);
				}

				// rx
				if (rx < DEAD_ZONE && rx > -DEAD_ZONE) {
					rx = 0;
				}
				else if (rx >= DEAD_ZONE) {
					rx = (rx - DEAD_ZONE) * 32767 / (32767 - DEAD_ZONE);
				}
				else {
					rx = (rx + DEAD_ZONE) * 32768 / (32768 - DEAD_ZONE);
				}

				// ry
				if (ry < DEAD_ZONE && ry > -DEAD_ZONE) {
					ry = 0;
				}
				else if (ry >= DEAD_ZONE) {
					ry = (ry - DEAD_ZONE) * 32767 / (32767 - DEAD_ZONE);
				}
				else {
					ry = (ry + DEAD_ZONE) * 32768 / (32768 - DEAD_ZONE);
				}

				std::basic_ostringstream<TCHAR> oss;
				oss << _T("LX: ") << lx << _T(" LY: ") << ly;
				oss << _T(" RX: ") << rx << _T(" RY: ") << ry;
				oss << std::endl;
				OutputDebugString(oss.str().c_str());

				unsigned short xbc_digital = 0;
				if (x.wButtons & XINPUT_GAMEPAD_BACK)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: BACK BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_BACK;
				}
				if (x.wButtons & XINPUT_GAMEPAD_START)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: START BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_START;
				}
				if (x.wButtons & XINPUT_GAMEPAD_A)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: A BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_A;
				}
				if (x.wButtons & XINPUT_GAMEPAD_B)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: B BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_B;
				}
				if (x.wButtons & XINPUT_GAMEPAD_X)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: X BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_X;
				}
				if (x.wButtons & XINPUT_GAMEPAD_Y)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: Y BUTTON");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_Y;
				}
				if (x.wButtons & XINPUT_GAMEPAD_DPAD_UP)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: DPAD UP");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_UP;
				}
				if (x.wButtons & XINPUT_GAMEPAD_DPAD_DOWN)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: DPAD DOWN");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_DOWN;
				}
				if (x.wButtons & XINPUT_GAMEPAD_DPAD_LEFT)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: DPAD LEFT");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_LEFT;
				}
				if (x.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: DPAD RIGHT");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_RIGHT;
				}
				if (x.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: LEFT BUMPER PRESSED");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_LB;
				}
				if (x.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: RIGHT BUMPER PRESSED");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_RB;
				}
				if (x.wButtons & XINPUT_GAMEPAD_LEFT_THUMB)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: LEFT JOYSTICK PRESSED");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_L_JOY;
				}
				if (x.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB)
				{
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX: RIGHT JOYSTICK PRESSED");
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
					xbc_digital |= XBC_R_JOY;
				}
				if (lx != 0 || ly != 0 || rx != 0 || ry != 0) {
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX | LX: ") << lx << _T(" LY: ") << ly << (" RX : ") << rx << _T(" RY : ") << ry;
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
				}

				if (lt != 0 || rt != 0) {
					std::basic_ostringstream<TCHAR> string_to_write;
					string_to_write << _T("Sent XBOX | LT: ") << std::setfill(_T('0')) << std::setw(3) << lt << _T(" RT: ") << std::setfill(_T('0')) << std::setw(3) << rt;
					if (AfxGetApp() && (CMainFrame*)AfxGetApp()->GetMainWnd()) {
						AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
					}
				}
				if (serial && (*serial) && (*serial)->is_connected()) {
					(*serial)->write(RobotMCtrl().xbox_keys_part1(xbc_digital, lt, rt, lx, ly));
				}
				if (serial && (*serial) && (*serial)->is_connected()) {
					(*serial)->write(RobotMCtrl().xbox_keys_part2(rx, ry));
				}
			}
			Sleep(10);
		}
		delete controller;
		controller = NULL;
	}
	return 0;
};