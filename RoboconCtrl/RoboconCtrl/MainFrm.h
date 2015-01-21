
// MainFrm.h : interface of the CMainFrame class
//

#pragma once
#include "OutputWnd.h"
#include "PropertiesWnd.h"
#include "InputWnd.h"
#include "SerialIO.h"

class CMainFrame : public CFrameWndEx
{
	
protected: // create from serialization only
	CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	virtual BOOL LoadFrame(UINT nIDResource, DWORD dwDefaultStyle = WS_OVERLAPPEDWINDOW | FWS_ADDTOTITLE, CWnd* pParentWnd = NULL, CCreateContext* pContext = NULL);

// Implementation
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif
	std::vector<std::basic_string<TCHAR>> GetSettings();
	void print_to_output(std::basic_string<TCHAR> string_to_print);
	void print_from_serial(std::basic_string<TCHAR> string_to_print);

	void print_read_from_serial(std::string string_to_print);
	void print_write_to_serial(std::basic_string<TCHAR> string_to_print, int prefix_enable);

	BOOL get_pid_mode();

protected:  // control bar embedded members
	CMFCMenuBar       m_wndMenuBar;
	CMFCToolBar       m_wndToolBar;
	CMFCStatusBar     m_wndStatusBar;
	CMFCToolBarImages m_UserImages;
	COutputWnd        m_wndOutput;
	CPropertiesWnd    m_wndProperties;
	CInputWnd         m_wndInput;

public: 
// serial port member
	static SerialIO* serial;
protected:
// buffer that stores keys pressed
	std::string send_msg;
	BOOL shift;
public:

// Generated message map functions
protected:
	afx_msg void OnEditCopy();
	afx_msg void OnEditPaste();
	afx_msg void OnEditClear();
	afx_msg void OnEditSelectAll();
	afx_msg void OpenConnection();
	afx_msg void CloseConnection();
	afx_msg LRESULT WriteString(WPARAM w, LPARAM l);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnViewCustomize();
	afx_msg LRESULT OnToolbarCreateNew(WPARAM wp, LPARAM lp);
	afx_msg void OnSettingChange(UINT uFlags, LPCTSTR lpszSection);
	afx_msg void OnUpdateFileNew(CCmdUI *pCmdUI);
	afx_msg void OnUpdateFileSave(CCmdUI *pCmdUI);
	afx_msg BOOL PreTranslateMessage(MSG* msg);
	afx_msg LRESULT print_read_from_serial(WPARAM w, LPARAM l);
	afx_msg LRESULT print_write_to_serial(WPARAM w, LPARAM l);

// Thread functions
protected:
	CWinThread* threads[2];
	static UINT __cdecl read_thread(LPVOID app_ptr);
	static UINT __cdecl write_thread(LPVOID app_ptr);
	DECLARE_MESSAGE_MAP()

	BOOL CreateDockingWindows();
	void SetDockingWindowIcons(BOOL bHiColorIcons);
};