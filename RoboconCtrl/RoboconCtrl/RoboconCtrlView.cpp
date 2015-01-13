
// RoboconCtrlView.cpp : implementation of the CRoboconCtrlView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "RoboconCtrl.h"
#endif

#include "RoboconCtrlDoc.h"
#include "RoboconCtrlView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include <gl/GL.h>
#include <gl/GLU.h>
#include <sstream>

// CRoboconCtrlView

IMPLEMENT_DYNCREATE(CRoboconCtrlView, CView)

BEGIN_MESSAGE_MAP(CRoboconCtrlView, CView)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_ERASEBKGND()
END_MESSAGE_MAP()

// CRoboconCtrlView construction/destruction

CRoboconCtrlView::CRoboconCtrlView()
{
	// TODO: add construction code here

}

CRoboconCtrlView::~CRoboconCtrlView()
{
}

BOOL CRoboconCtrlView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	cs.style |= (WS_CLIPCHILDREN | WS_CLIPSIBLINGS | CS_OWNDC);
	return CView::PreCreateWindow(cs);
}

BOOL CRoboconCtrlView::OnEraseBkgnd(CDC* pDC)
{
	return FALSE;
}

int CRoboconCtrlView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1) {
		return -1;
	}

	int pixel_format; // index of pixel format
	m_hDC = ::GetDC(m_hWnd);
	static PIXELFORMATDESCRIPTOR p = {
		sizeof(PIXELFORMATDESCRIPTOR), // size of struct
		1, // struct version
		PFD_DRAW_TO_WINDOW | //draw to window
		PFD_SUPPORT_OPENGL | //support opengl
		PFD_DOUBLEBUFFER, //double buffer mode
		PFD_TYPE_RGBA, //color mode rgba
		24, //24 bit color
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //options not used (13 0s)
		32, //size of depth buffer
		0, 0, //options not used (2 0s)
		PFD_MAIN_PLANE,
		0, 0, 0, 0 // options not used
	};
	// Choose the pixel format
	pixel_format = ChoosePixelFormat(m_hDC, &p);

	// set pixel format and verify that it is valid
	VERIFY(SetPixelFormat(m_hDC, pixel_format, &p));
	m_hRC = wglCreateContext(m_hDC);

	// make rendering context current, the initializes and deselects it
	VERIFY(wglMakeCurrent(m_hDC, m_hRC));

	// Set clearing background color 
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	SetTimer(1, 1, 0);

	OnDraw(NULL);

	return 0;
}

void CRoboconCtrlView::OnDestroy()
{
	wglMakeCurrent(m_hDC, NULL);
	wglDeleteContext(m_hRC);
	CView::OnDestroy();
}

// CRoboconCtrlView drawing

void CRoboconCtrlView::GLDrawScene()
{
	glColor3f((127.0f/255.0f), (51.0f/255.0f), 1.0f);
	glBegin(GL_QUADS);
		// Rectangle
		glVertex2f(-0.8f, -0.8f);
		glVertex2f(-0.8f, 0.8f);
		glVertex2f(0.8f, 0.8f);
		glVertex2f(0.8f, -0.8f);
	glEnd();
}

void CRoboconCtrlView::OnDraw(CDC* /*pDC*/)
{
	CRoboconCtrlDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLDrawScene();
	SwapBuffers(m_hDC);
}

void CRoboconCtrlView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	OnContextMenu(this, point);
}

void CRoboconCtrlView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
//	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif

	CRect rect;
	GetClientRect(&rect);

	std::basic_ostringstream<TCHAR> oss;
	oss << _T("WIDTH: ") << rect.Size().cx << _T(" HEIGHT: ") << rect.Size().cy;
	OutputDebugString(oss.str().c_str());
}


// CRoboconCtrlView diagnostics

#ifdef _DEBUG
void CRoboconCtrlView::AssertValid() const
{
	CView::AssertValid();
}

void CRoboconCtrlView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CRoboconCtrlDoc* CRoboconCtrlView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRoboconCtrlDoc)));
	return (CRoboconCtrlDoc*)m_pDocument;
}
#endif //_DEBUG

// CRoboconCtrlView message handlers

void CRoboconCtrlView::OnSize(UINT nType, int cx, int cy)
{
	gl_height = cy;
	gl_width = gl_height;
	glViewport((cx - gl_width) / 2, (cy - gl_height) / 2, gl_width, gl_height);
	CView::OnSize(nType, cx, cy);
}