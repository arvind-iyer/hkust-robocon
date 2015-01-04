
// Robocon Control PanelView.cpp : implementation of the CRoboconControlPanelView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "Robocon Control Panel.h"
#endif

#include "Robocon Control PanelDoc.h"
#include "Robocon Control PanelView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CRoboconControlPanelView

IMPLEMENT_DYNCREATE(CRoboconControlPanelView, CView)

BEGIN_MESSAGE_MAP(CRoboconControlPanelView, CView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
END_MESSAGE_MAP()

// CRoboconControlPanelView construction/destruction

CRoboconControlPanelView::CRoboconControlPanelView()
{
	// TODO: add construction code here

}

CRoboconControlPanelView::~CRoboconControlPanelView()
{
}

BOOL CRoboconControlPanelView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CRoboconControlPanelView drawing

void CRoboconControlPanelView::OnDraw(CDC* /*pDC*/)
{
	CRoboconControlPanelDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}

void CRoboconControlPanelView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	OnContextMenu(this, point);
}

void CRoboconControlPanelView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CRoboconControlPanelView diagnostics

#ifdef _DEBUG
void CRoboconControlPanelView::AssertValid() const
{
	CView::AssertValid();
}

void CRoboconControlPanelView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CRoboconControlPanelDoc* CRoboconControlPanelView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRoboconControlPanelDoc)));
	return (CRoboconControlPanelDoc*)m_pDocument;
}
#endif //_DEBUG


// CRoboconControlPanelView message handlers
