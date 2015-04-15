
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
	afx_msg void OnAppAbout();

	DECLARE_MESSAGE_MAP()
};

extern CRoboconCtrlApp theApp;
