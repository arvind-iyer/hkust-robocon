
// Robocon Control PanelView.h : interface of the CRoboconControlPanelView class
//

#pragma once


class CRoboconControlPanelView : public CView
{
protected: // create from serialization only
	CRoboconControlPanelView();
	DECLARE_DYNCREATE(CRoboconControlPanelView)

// Attributes
public:
	CRoboconControlPanelDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CRoboconControlPanelView();
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

#ifndef _DEBUG  // debug version in Robocon Control PanelView.cpp
inline CRoboconControlPanelDoc* CRoboconControlPanelView::GetDocument() const
   { return reinterpret_cast<CRoboconControlPanelDoc*>(m_pDocument); }
#endif

