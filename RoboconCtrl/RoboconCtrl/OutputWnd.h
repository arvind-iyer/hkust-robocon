
#pragma once

#include <string>

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

// Stores horizontal extent
public:
	int cxExtentMax;

protected:
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	afx_msg void OnViewOutput();
	afx_msg void OnEditCopy();
	afx_msg void OnEditClear();

	DECLARE_MESSAGE_MAP()
};

class COutputWnd : public CDockablePane
{

// Construction
public:
	COutputWnd();
//	void PrintString(std::basic_string<TCHAR> string_to_print); //Prints only to all logs
//	void ReadFromSerial(std::basic_string<TCHAR> string_read_from_serial); //Prints to serial and all logs
	void UpdateFonts();

// Attributes
public:
	CMFCTabCtrl	m_wndTabs;
	COutputList m_wndOutputBuild;
	COutputList m_wndOutputRead;
	COutputList m_wndOutputDebug;

protected:
	void AdjustHorzScroll(COutputList& wndListBox);

// Implementation
public:
	virtual ~COutputWnd();

protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg LRESULT ReadFromSerial(WPARAM w, LPARAM l);
	afx_msg LRESULT ReadDebugFromSerial(WPARAM w, LPARAM l);
	afx_msg LRESULT PrintString(WPARAM w, LPARAM l);

	DECLARE_MESSAGE_MAP()
};

