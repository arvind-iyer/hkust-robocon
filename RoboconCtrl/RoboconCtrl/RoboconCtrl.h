
// RoboconCtrl.h : main header file for the RoboconCtrl application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols

// Serial I/O Class
#include "SerialIO.h"

// CRoboconCtrlApp:
// See RoboconCtrl.cpp for the implementation of this class
//

class CRoboconCtrlApp : public CWinAppEx
{
private:
	static SerialIO* serial;

public:
	CRoboconCtrlApp();
	~CRoboconCtrlApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	BOOL  m_bHiColorIcons;

	virtual void PreLoadState();

	static void __cdecl read_thread(void* ioport);

	void PrintOutput(std::string string, int output_number);
	void PrintOutput(std::wstring string, int output_number);

	afx_msg void OpenConnection();
	afx_msg void CloseConnection();
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CRoboconCtrlApp theApp;
