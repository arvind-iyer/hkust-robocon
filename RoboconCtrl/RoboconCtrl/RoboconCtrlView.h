
// RoboconCtrlView.h : interface of the CRoboconCtrlView class
//

#pragma once


class CRoboconCtrlView : public CView
{
protected: // create from serialization only
	CRoboconCtrlView();
	DECLARE_DYNCREATE(CRoboconCtrlView)

// Attributes
public:
	CRoboconCtrlDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CRoboconCtrlView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	afx_msg void OnFilePrintPreview();
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in RoboconCtrlView.cpp
inline CRoboconCtrlDoc* CRoboconCtrlView::GetDocument() const
   { return reinterpret_cast<CRoboconCtrlDoc*>(m_pDocument); }
#endif

