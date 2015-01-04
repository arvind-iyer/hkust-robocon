
// Robocon Control Panel.h : main header file for the Robocon Control Panel application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CRoboconControlPanelApp:
// See Robocon Control Panel.cpp for the implementation of this class
//

class CRoboconControlPanelApp : public CWinAppEx
{
public:
	CRoboconControlPanelApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	UINT  m_nAppLook;
	BOOL  m_bHiColorIcons;

	virtual void PreLoadState();
	virtual void LoadCustomState();
	virtual void SaveCustomState();

	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CRoboconControlPanelApp theApp;
