#pragma once

class CInputBox : public CEdit
{
	// Construction
public:
	CInputBox();

	// Implementation
public:
	virtual ~CInputBox();

protected:
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	afx_msg void OnEditCopy();
	afx_msg void OnEditPaste();
	afx_msg void OnEditSelectAll();
	afx_msg void OnViewOutput();
	afx_msg void OnEditClear();
	afx_msg void OnChar(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg BOOL PreTranslateMessage(MSG* pMsg);

	DECLARE_MESSAGE_MAP()
};

// CInputWnd

class CInputWnd : public CDockablePane
{
	DECLARE_DYNAMIC(CInputWnd)

public:
	CInputWnd();
	virtual ~CInputWnd();

// Attributes
public:
	CInputBox m_wndInputBox;

protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);

	DECLARE_MESSAGE_MAP()
};


