
#pragma once

#include "SerialIO.h"

/////////////////////////////////////////////////////////////////////////////
// COutputList window

class COutputList : public CListBox
{
// Construction
public:
	COutputList();

// Implementation
public:
	virtual ~COutputList();

protected:
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	afx_msg void OnEditCopy();
	afx_msg void OnEditClear();
	afx_msg void OnViewOutput();

	DECLARE_MESSAGE_MAP()
};

class COutputWnd : public CDockablePane
{
	SerialIO* serial_wnd;

// Construction
public:
	COutputWnd();
	void PrintString(std::basic_string<TCHAR> string_to_print); //Prints only to all logs
	void ReadFromSerial(std::basic_string<TCHAR> string_read_from_serial); //Prints to serial and all logs
	void UpdateFonts();

// Attributes
protected:
	CMFCTabCtrl	m_wndTabs;

	COutputList m_wndOutputBuild;
	COutputList m_wndOutputRead;

protected:
	void FillBuildWindow();

	void AdjustHorzScroll(CListBox& wndListBox);

// Implementation
public:
	virtual ~COutputWnd();

protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);

	DECLARE_MESSAGE_MAP()
};

