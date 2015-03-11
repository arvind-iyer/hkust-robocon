#include "stdafx.h"
#include "CXBOXController.h"
#include "SerialIO.h"
#include "MainFrm.h"
#include "RobotMCtrl.h"

#include <sstream>
#include <XInput.h>

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
	CXBOXController* controller = NULL;
	// wait for xbox to be connected
	while (running) {
		while (controller == NULL && running)
		{
			if (AfxGetApp() && ((CMainFrame*)AfxGetApp()->GetMainWnd())) {
				(((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())->PostMessageW(UWM_RECEIVE_XBOX, 0, 0);
			}
			controller = new CXBOXController(1);
			if (!controller->is_connected()) {
				delete controller;
				controller = NULL;
			}
			Sleep(50);
		}

		while (controller->is_connected() && running)
		{
			if (AfxGetApp() && ((CMainFrame*)AfxGetApp()->GetMainWnd())) {
				(((CMainFrame*)AfxGetApp()->GetMainWnd())->GetActiveView())->PostMessageW(UWM_RECEIVE_XBOX, 0, 1);
			}
			XINPUT_GAMEPAD x = controller->GetState().Gamepad;
			short lx = x.sThumbLX, ly = x.sThumbLY, rx = x.sThumbRX, ry = x.sThumbRY;

			// scale to 100

			// dead zone correction
			
			// lx
			if (lx < DEAD_ZONE && lx > -DEAD_ZONE) {
				lx = 0;
			}
			else if (lx >= DEAD_ZONE) {
				lx = (lx - DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}
			else {
				lx = (lx + DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}

			// ly
			if (ly < DEAD_ZONE && ly > -DEAD_ZONE) {
				ly = 0;
			}
			else if (ly >= DEAD_ZONE) {
				ly = (ly - DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}
			else {
				ly = (ly + DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}

			// rx
			if (rx < DEAD_ZONE && rx > -DEAD_ZONE) {
				rx = 0;
			}
			else if (rx >= DEAD_ZONE) {
				rx = (rx - DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}
			else {
				rx = (rx + DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}

			// ry
			if (ry < DEAD_ZONE && ry > -DEAD_ZONE) {
				ry = 0;
			}
			else if (ry >= DEAD_ZONE) {
				ry = (ry - DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}
			else {
				ry = (ry + DEAD_ZONE) * 100 / (32767 - DEAD_ZONE);
			}
			
			std::basic_ostringstream<TCHAR> oss;
			oss << _T("LX: ") << lx << _T(" LY: ") << ly;
			oss << _T(" RX: ") << rx << _T(" RY: ") << ry;
			oss << std::endl;
			OutputDebugString(oss.str().c_str());
			if (*serial != NULL && running) {
				std::basic_ostringstream<TCHAR> string_to_write;
				string_to_write << _T("Sent Controls | X: ") << lx << _T(" Y: ") << ly << _T(" w: ") << rx;
				(*serial)->write(RobotMCtrl().manual_mode(lx, ly, rx));
				if (AfxGetApp() && AfxGetApp()->GetMainWnd()) {
					AfxGetApp()->GetMainWnd()->PostMessage(UWM_PRINT_OUTPUT_FROM_WRITE, 0, (LPARAM)new std::basic_string<TCHAR>(string_to_write.str()));
				}
			}
			Sleep(10);
		}
		delete controller;
		controller = NULL;
	}
	return 0;
};