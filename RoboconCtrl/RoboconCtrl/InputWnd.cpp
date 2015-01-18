// InputWnd.cpp : implementation file
//

#include "stdafx.h"
#include "RoboconCtrl.h"
#include "InputWnd.h"
#include "MainFrm.h"
#include <vector>
#include <sstream>
#include <string>

// CInputWnd

IMPLEMENT_DYNAMIC(CInputWnd, CDockablePane)

CInputWnd::CInputWnd()
{

}

CInputWnd::~CInputWnd()
{
}


BEGIN_MESSAGE_MAP(CInputWnd, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
END_MESSAGE_MAP()

int CInputWnd::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;
	DWORD style =  ES_MULTILINE | WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER;
	CRect rectDummy;
	rectDummy.SetRectEmpty();
	if (!m_wndInputBox.Create(style, rectDummy, this, ID_INPUT_BOX))
		return -1;
	return 0;
}

void CInputWnd::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType, cx, cy);
	m_wndInputBox.SetWindowPos(NULL, -1, -1, cx, cy, SWP_NOMOVE | SWP_NOACTIVATE | SWP_NOZORDER);
}

// CInputWnd message handlers

// CInputBox

CInputBox::CInputBox()
{

}

CInputBox::~CInputBox()
{

}

void CInputBox::OnEditCopy()
{
	Copy();
}

void CInputBox::OnEditPaste()
{
	Paste();
}

void CInputBox::OnEditSelectAll()
{
	SetSel(0, -1);
}

void CInputBox::OnEditClear()
{
	SetSel(0, -1);
	Clear();
}

BOOL CInputBox::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN && pMsg->wParam == VK_RETURN && GetFocus() == this)
	{
		CString write_string;
		GetWindowText(write_string);
		OutputDebugString(write_string);
		AfxGetMainWnd()->PostMessage(WM_SEND_STRING, 1,(LPARAM)new CString(write_string));
		SetSel(0, -1);
		Clear();
		return TRUE; // this doesn't need processing anymore
	}
	return FALSE; // all other cases still need default processing
}

void CInputBox::OnChar(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	std::basic_ostringstream<TCHAR> oss;
	oss << (TCHAR)nChar;
	if (((CMainFrame*)AfxGetMainWnd())->GetSettings()[3] == _T("1")) {
		AfxGetMainWnd()->PostMessage(WM_SEND_STRING, 1, (LPARAM)(new CString(oss.str().c_str())));
	}
	OutputDebugString(oss.str().c_str());
	CEdit::OnChar(nChar, nRepCnt, nFlags);
}

BEGIN_MESSAGE_MAP(CInputBox, CEdit)
	ON_COMMAND(ID_EDIT_COPY, OnEditCopy)
	ON_COMMAND(ID_EDIT_PASTE, OnEditPaste)
	ON_COMMAND(ID_EDIT_SELECT_ALL, OnEditSelectAll)
	ON_COMMAND(ID_EDIT_CLEAR,OnEditClear)
	ON_WM_CHAR()
END_MESSAGE_MAP()
