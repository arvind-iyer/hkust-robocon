
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
	ON_WM_LBUTTONDBLCLK()
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
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
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
	glBegin(GL_QUADS);
		glColor3f((103.0f / 255.0f), (196.0f / 255.0f), (100.0f / 255.0f));
		// Rectangle for game field
		glVertex2f(-1.0f, -1.0f);
		glVertex2f(-1.0f, 1.0f);
		glVertex2f(1.0f, 1.0f);
		glVertex2f(1.0f, -1.0f);

		glColor3f((255.0f / 255.0f), (196.0f / 255.0f), (100.0f / 255.0f));

		// Serving zones
		glVertex2f(3.945f / 6.7f, 0.55f / 3.15f);
		glVertex2f(3.945f / 6.7f, 2.57f / 3.15f);
		glVertex2f(5.195f / 6.7f, 2.57f / 3.15f);
		glVertex2f(5.195f / 6.7f, 0.55f / 3.15f);

		glVertex2f(-3.945f / 6.7f, -0.55f / 3.15f);
		glVertex2f(-3.945f / 6.7f, -2.57f / 3.15f);
		glVertex2f(-5.195f / 6.7f, -2.57f / 3.15f);
		glVertex2f(-5.195f / 6.7f, -0.55f / 3.15f);
	glEnd();

	// Drawing game field lines
	glBegin(GL_LINES);
		glColor3f(0.0f, 0.0f, 0.0f);

		// Serving lines
		glVertex2f(2.0f / 6.7f, 1.0f);
		glVertex2f(2.0f / 6.7f, -1.0f);
		glVertex2f(-2.0f / 6.7f, 1.0f);
		glVertex2f(-2.0f / 6.7f, -1.0f);

		// Long service lines for doubles
		glVertex2f(5.92f / 6.7f, 1.0f);
		glVertex2f(5.92f / 6.7f, -1.0f);
		glVertex2f(-5.92f / 6.7f, 1.0f);
		glVertex2f(-5.92f / 6.7f, -1.0f);

		// Center lines
		glVertex2f(-1.0f, 0.0f);
		glVertex2f(-2.0f / 6.7f, 0.0f);
		glVertex2f(1.0f, 0.0f);
		glVertex2f(2.0f / 6.7f, 0.0f);

		// Side lines for singles
		glVertex2f(-1.0f, 2.57f / 3.15f);
		glVertex2f(1.0f, 2.57f / 3.15f);
		glVertex2f(-1.0f, -2.57f / 3.15f);
		glVertex2f(1.0f, -2.57f / 3.15f);

		glColor3f(1.0f, 0.0f, 0.0f);

		glLineWidth(2.0f);

		// Net
		glVertex2f(0.0f, -1.0f);
		glVertex2f(0.0f, 1.0f);
		
		glLineWidth(1.0f);

	glEnd();

	glLineWidth(2.0f);
	glBegin(GL_LINE_LOOP);
		// Game field border
		glVertex2f(-1.0f, 1.0f);
		glVertex2f(1.0f, 1.0f);
		glVertex2f(1.0f, -1.0f);
		glVertex2f(-1.0f, -1.0f);
	glEnd();

	glLineWidth(1.0f);
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

void CRoboconCtrlView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	std::basic_ostringstream<TCHAR> oss;
	oss << _T("X: ") << point.x << _T(" Y: ") << point.y;
	OutputDebugString(oss.str().c_str());
	CView::OnLButtonDblClk(nFlags, point);
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
	double ratio = 61.0 / 134.0;
	if ((cx * ratio) > (double)(cy)) {
		gl_height = cy * 90 / 100;
		gl_width = (int)round(gl_height / ratio);
	}
	else {
		gl_width = cx * 90 /100;
		gl_height = (int)round(gl_width * ratio);
	}
	glViewport((cx - gl_width) / 2, (cy - gl_height) / 2, gl_width, gl_height);
	CView::OnSize(nType, cx, cy);
}