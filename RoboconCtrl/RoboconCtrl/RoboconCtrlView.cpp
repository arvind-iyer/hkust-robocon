
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
#include "MainFrm.h"

#include "RobotMCtrl.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include <gl/GL.h>
#include <gl/GLU.h>
#include <sstream>
#include <iomanip>
#include <memory>

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
	ON_WM_MOUSEMOVE()
	ON_MESSAGE(WM_RECEIVE_ROBOT_COORD, refresh_coordinates)
END_MESSAGE_MAP()

// CRoboconCtrlView construction/destruction

CRoboconCtrlView::CRoboconCtrlView()
{
	// TODO: add construction code here
	current_pos.x = 0.0f;
	current_pos.y = 0.0f;
	current_pos.angle = 0;
	current_pos.valid = FALSE;

	cursor_pos.x = 0.0f;
	cursor_pos.y = 0.0f;
	cursor_pos.angle = 0;
	cursor_pos.valid = FALSE;

	robot_pos.x = 0.0f;
	robot_pos.y = 0.0f;
	robot_pos.angle = 0;
	robot_pos.valid = FALSE;

	grid_pos.valid = FALSE;
}

CRoboconCtrlView::~CRoboconCtrlView()
{
}

BOOL CRoboconCtrlView::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN) {
		if (GetAsyncKeyState(VK_TAB) & 0x8000) {
			cursor_pos.angle == 0 ? cursor_pos.angle = 358 : cursor_pos.angle -= 2;
			Invalidate();
		}
		if (GetAsyncKeyState(0x52) & 0x8000) {
			cursor_pos.angle >= 358 ? cursor_pos.angle = 0 : cursor_pos.angle += 2;
			Invalidate();
		}
	}
	return CWnd::PreTranslateMessage(pMsg);
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

CRoboconCtrlView::GLCoord CRoboconCtrlView::GetGLCoord(CPoint wndCoord)
{
	CRect rect;
	GetClientRect(rect);
	GLCoord g;
	g.x = -(float)(((double)rect.Width() / 2 - (double)(wndCoord.x + 1)) / ((double)gl_width / 2));
	g.y = (float)(((double)rect.Height() / 2 - (double)(wndCoord.y + 1)) / ((double)gl_height / 2));

	if (g.x > 1.0f || g.y > 1.0f) {
		g.valid = FALSE;
	}
	else {
		g.valid = TRUE;
	}

	return g;
}

CRoboconCtrlView::GridCoord CRoboconCtrlView::GetRobotCoord(CPoint wndCoord, unsigned int angle) {
	CRect rect;
	GetClientRect(rect);

	int x, y = 0;

	if (wndCoord.x < rect.Width() / 2) {
		x = (-3050 * (rect.Height() - 2 * (wndCoord.y + 1))) / gl_height;
		y = (-6700 * (rect.Width() - 2 * wndCoord.x - gl_width - 2)) / gl_width;
	}
	else {
		x = (3050 * (rect.Height() - 2 * (wndCoord.y + 1))) / gl_height;
		y = (6700 * (rect.Width() - 2 * wndCoord.x + gl_width - 2)) / gl_width;
	}
	GridCoord g;
	g.x = x;
	g.y = y;
	g.angle = angle;
	if (x > 3050 || x < -3050 || y < 0) {
		g.valid = FALSE;
	}
	else {
		g.valid = TRUE;
	}
	return g;
}

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
		glVertex2f(3.945f / 6.7f, 0.55f / 3.05f);
		glVertex2f(3.945f / 6.7f, 2.57f / 3.05f);
		glVertex2f(5.195f / 6.7f, 2.57f / 3.05f);
		glVertex2f(5.195f / 6.7f, 0.55f / 3.05f);

		glVertex2f(-3.945f / 6.7f, -0.55f / 3.05f);
		glVertex2f(-3.945f / 6.7f, -2.57f / 3.05f);
		glVertex2f(-5.195f / 6.7f, -2.57f / 3.05f);
		glVertex2f(-5.195f / 6.7f, -0.55f / 3.05f);
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
		glVertex2f(-1.0f, 2.57f / 3.05f);
		glVertex2f(1.0f, 2.57f / 3.05f);
		glVertex2f(-1.0f, -2.57f / 3.05f);
		glVertex2f(1.0f, -2.57f / 3.05f);

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
	// Draw current position
	if (current_pos.valid) {
		// calculate position
		double length = 0.0005;
		double angle_of_quad = 40.0 * std::atan2(0, -1) / 180.0;

		double angle = current_pos.angle * std::atan2(0, -1) / 180.0;

		double quad_xpos1 = 61.0 * (cos(-angle) * -length) + current_pos.x;
		double quad_ypos1 = 134.0 * (sin(-angle) * -length) + current_pos.y;

		double quad_xpos2 = 61.0 * (cos(-angle) * length * cos(angle_of_quad / 2) - sin(-angle) * length * sin(angle_of_quad / 2)) + current_pos.x;
		double quad_ypos2 = 134.0 * (sin(-angle) * length * cos(angle_of_quad / 2) + cos(-angle) * length * sin(angle_of_quad / 2)) + current_pos.y;

		double quad_xpos3 = 61.0 * (cos(-angle) * length * (cos(angle_of_quad / 2) - sin(angle_of_quad / 2))) + current_pos.x;
		double quad_ypos3 = 134.0 * (sin(-angle) * length * (cos(angle_of_quad / 2) - sin(angle_of_quad / 2))) + current_pos.y;

		double quad_xpos4 = 61.0 * (cos(-angle) * length * cos(angle_of_quad / 2) - sin(-angle) * -length * sin(angle_of_quad / 2)) + current_pos.x;
		double quad_ypos4 = 134.0 * (sin(-angle) * length * cos(angle_of_quad / 2) + cos(-angle) * -length * sin(angle_of_quad / 2)) + current_pos.y;

		std::basic_ostringstream<TCHAR> oss;
		oss << _T("Coords: (") << quad_xpos1 << _T(", ") << quad_ypos1 << _T(") (") << quad_xpos2 << _T(", ") << quad_ypos2 << _T(") (") << quad_xpos3 << _T(", ") << quad_ypos3 << _T(") (") << quad_xpos4 << _T(", ") << quad_ypos4 << _T(")") << std::endl;

		OutputDebugString(oss.str().c_str());

		glBegin(GL_TRIANGLE_FAN);
			glColor3f((92.0f / 255.0f), (196.0f / 255.0f), (255.0f / 255.0f));
			glVertex2d(quad_xpos1, quad_ypos1);
			glColor3f((0.0f / 255.0f), (0.0f / 255.0f), (255.0f / 255.0f));
			glVertex2d(quad_xpos2, quad_ypos2);
			glVertex2d(quad_xpos3, quad_ypos3);
			glVertex2d(quad_xpos4, quad_ypos4);
		glEnd();
		glPointSize(3.0f);
		glBegin(GL_POINTS);
			glColor3f(0.0f, 0.0f, 0.0f);
			glVertex2d(current_pos.x, current_pos.y);
		glEnd();
	}
	if (robot_pos.valid) {
		// std::atan2(0, -1) is pi
		double angle = std::atan2(0, -1) * robot_pos.angle / 180.0;
		double side_length = 0.07;

		// Rotation equations
		double triangle_xpos1 = 61.0 * (cos(-angle) * (-side_length * sqrt(3) / 3)) + robot_pos.x;
		double triangle_ypos1 = 134.0 * (sin(-angle) * (-side_length * sqrt(3) / 3)) + robot_pos.y;

		double triangle_xpos2 = 61.0 * (cos(-angle) * (side_length * sqrt(3) / 6) - sin(-angle) * (side_length * 0.5)) + robot_pos.x;
		double triangle_ypos2 = 134.0 * (sin(-angle) * (side_length * sqrt(3) / 6) + cos(-angle) * (side_length * 0.5)) + robot_pos.y;

		double triangle_xpos3 = 61.0 * (cos(-angle) * (side_length * sqrt(3) / 6) - sin(-angle) * (-side_length * 0.5)) + robot_pos.x;
		double triangle_ypos3 = 134.0 * (sin(-angle) * (side_length * sqrt(3) / 6) + cos(-angle) * (-side_length * 0.5)) + robot_pos.y;

		glBegin(GL_TRIANGLES);
			// width 0.0005
			glColor3f(3.0f / 255.0f, 78.0f / 255.0f, 164.0f / 255.0f);
			glVertex2d(triangle_xpos1, triangle_ypos1);
			glColor3f(71.0f / 255.0f, 231.0f / 255.0f, 255.0f / 255.0f);
			glVertex2d(triangle_xpos2, triangle_ypos2);
			glVertex2d(triangle_xpos3, triangle_ypos3);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glColor3f(3.0f / 255.0f, 78.0f / 255.0f, 164.0f / 255.0f);
			glVertex2d(triangle_xpos1, triangle_ypos1);
			glVertex2d(triangle_xpos2, triangle_ypos2);
			glVertex2d(triangle_xpos3, triangle_ypos3);
		glEnd();
		glPointSize(3.0f);
		glBegin(GL_POINTS);
			glColor3f(205.0f / 255.0f, 0.0f, 205.0f / 255.0f);
			glVertex2f(robot_pos.x, robot_pos.y);
		glEnd();
	}

	if (cursor_pos.valid) {
		glBegin(GL_LINES);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex2f(cursor_pos.x + 0.0002f * 61, cursor_pos.y);
			glVertex2f(cursor_pos.x - 0.0002f * 61, cursor_pos.y);
			glVertex2f(cursor_pos.x, cursor_pos.y + 0.0002f * 134);
			glVertex2f(cursor_pos.x, cursor_pos.y - 0.0002f * 134);
		glEnd();
	}
}

void CRoboconCtrlView::OnDraw(CDC* pDC)
{
	CRoboconCtrlDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLDrawScene();
	SwapBuffers(m_hDC);

	if (grid_pos.valid) {
		std::basic_ostringstream<TCHAR> oss;
		oss << _T("Mouse coordinates: (") << grid_pos.x << _T(", ") << grid_pos.y << _T(", ) Angle: ") << cursor_pos.angle;
		TextOut(pDC->GetSafeHdc(), 1, 2, oss.str().c_str(), oss.str().size());
	}

	// get our client rectangle
	CRect rect;
	GetClientRect(rect);

	std::basic_string<TCHAR> status = std::basic_string<TCHAR>(_T("Comm Status: "));

	TextOut(pDC->GetSafeHdc(), rect.Width() - 115, 0, status.c_str(), status.size());

	CBrush* pOldBrush;
	CBrush brushRed(RGB(255, 0, 0));
	CBrush brushGreen(RGB(0, 255, 0));

	if (((CMainFrame*)AfxGetMainWnd())->serial != NULL && ((CMainFrame*)AfxGetMainWnd())->serial->is_connected()) {
		pOldBrush = pDC->SelectObject(&brushGreen);
	}
	else {
		pOldBrush = pDC->SelectObject(&brushRed);
	}

	// draw a thick black rectangle filled with blue
	pDC->Rectangle(rect.Width() - 20, 0, rect.Width(), 20);

	// put back the old objects
	pDC->SelectObject(pOldBrush);

}

void CRoboconCtrlView::OnMouseMove(UINT nFlags, CPoint point)
{
	GLCoord g = GetGLCoord(point);
	cursor_pos.x = g.x;
	cursor_pos.y = g.y;
	cursor_pos.valid = g.valid;
	grid_pos = GetRobotCoord(point);
	Invalidate();
}

void CRoboconCtrlView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	current_pos.valid = FALSE;
	Invalidate();
//	OnContextMenu(this, point);
}

void CRoboconCtrlView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
//	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif

	CRect rect;
	GetClientRect(&rect);
}

void CRoboconCtrlView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	CView::OnLButtonDblClk(nFlags, point);
	GLCoord g = GetGLCoord(point);

	current_pos.x = g.x;
	current_pos.y = g.y;
	current_pos.angle = cursor_pos.angle;
	current_pos.valid = g.valid;

	grid_pos.angle = cursor_pos.angle;

	if (grid_pos.valid){
		std::vector<short> data;
		data.push_back((short)grid_pos.x);
		data.push_back((short)grid_pos.y);
		data.push_back((unsigned short)grid_pos.angle);

		AfxGetMainWnd()->PostMessage(WM_SEND_STRING, 2, (LPARAM)new std::vector<short>(data));
	}
	Invalidate();
}

CRoboconCtrlView::GLCoord CRoboconCtrlView::ConvertGridCoordToGLCoord(GridCoord g)
{
	GLCoord r_pos;
	r_pos.x = (float)(1.0 - (g.y / 6700.0));
	r_pos.y = (float)(g.x / 3050.0);
	r_pos.angle = g.angle;
	r_pos.valid = g.valid;

	return r_pos;
}

LRESULT CRoboconCtrlView::refresh_coordinates(WPARAM w, LPARAM l) {
	std::shared_ptr< std::vector<int> > coordinates(reinterpret_cast< std::vector<int>* >(l));

	GridCoord r_pos;
	r_pos.x = (*coordinates)[0];
	r_pos.y = (*coordinates)[1];
	r_pos.angle = (*coordinates)[2];

	robot_pos = ConvertGridCoordToGLCoord(r_pos);
	Invalidate();
	return 0;
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