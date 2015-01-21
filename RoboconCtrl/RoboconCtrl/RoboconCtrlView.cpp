
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
#include <fstream>

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
	ON_MESSAGE(WM_RESET_ROBOT_POS, reset_coord)
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
	bool exist = TRUE;
	int i = 1;

	TCHAR path[MAX_PATH] = { 0 };
	GetModuleFileName(NULL, path, MAX_PATH);
	PathRemoveFileSpec(path);

	SetCurrentDirectory(path);

	while (exist) {
		std::basic_ostringstream<TCHAR> oss;
		oss << _T("data") << i << _T(".csv");
		if (GetFileAttributes(oss.str().c_str()) == INVALID_FILE_ATTRIBUTES) {
			if (!robot_path_data.empty()) {
				std::stringstream file_path;
				file_path << "data" << i << ".csv";

				std::ofstream output_data;
				output_data.open(file_path.str());
				for (size_t i = 0; i < robot_path_data.size(); ++i) {
					output_data << robot_path_data[i].x << ", " << robot_path_data[i].y << std::endl;
				}
				output_data.close();
			}
			exist = FALSE;
		}
		else {
			++i;
		}
	}
}

BOOL CRoboconCtrlView::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN) {
		if (GetAsyncKeyState(0x5A) & 0x8000) {
			cursor_pos.angle == 0 ? cursor_pos.angle = 358 : cursor_pos.angle -= 2;
			Invalidate();
		}
		if (GetAsyncKeyState(0x43) & 0x8000) {
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
		PFD_SUPPORT_GDI |
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

	OnDraw(GetDC());

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

void CRoboconCtrlView::DrawIndicator(GLCoord coordinates, GLColor point_ind_color, GLColor back_ind_color, GLColor point_color, int mode)
{
	// calculate position
	double length = 0.0007;
	double angle_of_quad = 40.0 * std::atan2(0, -1) / 180.0;

	double angle = coordinates.angle * std::atan2(0, -1) / 180.0;

	double quad_xpos1 = 61.0 * (cos(-angle) * -length) + coordinates.x;
	double quad_ypos1 = 134.0 * (sin(-angle) * -length) + coordinates.y;

	double quad_xpos2 = 61.0 * (cos(-angle) * length * cos(angle_of_quad / 2) - sin(-angle) * length * sin(angle_of_quad / 2)) + coordinates.x;
	double quad_ypos2 = 134.0 * (sin(-angle) * length * cos(angle_of_quad / 2) + cos(-angle) * length * sin(angle_of_quad / 2)) + coordinates.y;

	double quad_xpos3 = 61.0 * (cos(-angle) * length * (cos(angle_of_quad / 2) - sin(angle_of_quad / 2))) + coordinates.x;
	double quad_ypos3 = 134.0 * (sin(-angle) * length * (cos(angle_of_quad / 2) - sin(angle_of_quad / 2))) + coordinates.y;

	double quad_xpos4 = 61.0 * (cos(-angle) * length * cos(angle_of_quad / 2) - sin(-angle) * -length * sin(angle_of_quad / 2)) + coordinates.x;
	double quad_ypos4 = 134.0 * (sin(-angle) * length * cos(angle_of_quad / 2) + cos(-angle) * -length * sin(angle_of_quad / 2)) + coordinates.y;
	if (mode == 0) {
		glBegin(GL_TRIANGLE_FAN);
		glColor3f(point_ind_color.r, point_ind_color.g, point_ind_color.b);
		glVertex2d(quad_xpos1, quad_ypos1);
		glColor3f(back_ind_color.r, back_ind_color.g, back_ind_color.b);
		glVertex2d(quad_xpos2, quad_ypos2);
		glVertex2d(quad_xpos3, quad_ypos3);
		glVertex2d(quad_xpos4, quad_ypos4);
		glEnd();
	}
	glBegin(GL_LINE_LOOP);
		glColor3f(point_color.r, point_color.g, point_color.b);
		glVertex2d(quad_xpos1, quad_ypos1);
		glVertex2d(quad_xpos2, quad_ypos2);
		glVertex2d(quad_xpos3, quad_ypos3);
		glVertex2d(quad_xpos4, quad_ypos4);
	glEnd();
	glPointSize(2.0f);
	glBegin(GL_POINTS);
		glColor3f(point_color.r, point_color.g, point_color.b);
		glVertex2d(coordinates.x, coordinates.y);
	glEnd();
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
		GLColor point_ind = {
			92.0f / 255.0f,
			196.0f / 255.0f,
			255.0f / 255.0f
		};

		GLColor back_ind = {
			255.0f / 255.0f,
			255.0f / 255.0f,
			255.0f / 255.0f
		};

		GLColor point_color = {
			0.0f,
			0.0f,
			0.0f
		};
		DrawIndicator(current_pos, point_ind, back_ind, point_color);
	}
	if (robot_pos.valid) {
		GLColor point_ind = {
			92.0f / 255.0f,
			196.0f / 255.0f,
			255.0f / 255.0f
		};

		GLColor back_ind = {
			0.0f / 255.0f,
			0.0f / 255.0f,
			255.0f / 255.0f
		};

		GLColor point_color = {
			1.0f,
			1.0f,
			1.0f
		};
		DrawIndicator(robot_pos, point_ind, back_ind, point_color);
	}
	if (cursor_pos.valid) {
		GLColor point_ind = {};
		GLColor back_ind = {};
		GLColor point_color = {
			1.0f,
			0.0f,
			0.0f
		};
		DrawIndicator(cursor_pos, point_ind, back_ind, point_color, 1);
	}
	if (!robot_path.empty()) {
		if (robot_path.size() > 500) {
			robot_path.pop_front();
		}
		glBegin(GL_LINE_STRIP);
			glColor3f(238.0f / 255.0f, 252.0f / 255.0f, 73.0f / 255.0f);
			for (size_t i = 0; i < robot_path.size(); ++i) {
				glVertex2f(robot_path[i].x, robot_path[i].y);
			}
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
	glFinish();

	std::basic_ostringstream<TCHAR> oss;
	if (grid_pos.valid) {
		oss << _T("Mouse coordinates: (") << grid_pos.x << _T(", ") << grid_pos.y << _T(") ");
	}
	oss << _T("Angle: ") << cursor_pos.angle;
	TextOut(pDC->GetSafeHdc(), 1, 2, oss.str().c_str(), oss.str().size());

	// get our client rectangle
	CRect rect;
	GetClientRect(rect);
	std::basic_string<TCHAR> comm_status(_T("Comm Status: "));
	TextOut(pDC->GetSafeHdc(), rect.Width() - 115, 0, comm_status.c_str(), comm_status.size());

	std::basic_string<TCHAR> pid_status(_T("PID Status: "));
	TextOut(pDC->GetSafeHdc(), rect.Width() - 100, rect.Height() - 19, pid_status.c_str(), pid_status.size());

	CBrush* pOldBrush;
	CBrush brushRed(RGB(255, 0, 0));
	CBrush brushGreen(RGB(0, 255, 0));

	if (((CMainFrame*)AfxGetMainWnd())->serial != NULL && ((CMainFrame*)AfxGetMainWnd())->serial->is_connected()) {
		pOldBrush = pDC->SelectObject(&brushGreen);
	}
	else {
		pOldBrush = pDC->SelectObject(&brushRed);
	}

	pDC->Rectangle(rect.Width() - 20, 0, rect.Width(), 20);

	// put back the old objects
	pDC->SelectObject(pOldBrush);

	if (((CMainFrame*)AfxGetMainWnd())->get_pid_mode()) {
		pOldBrush = pDC->SelectObject(&brushGreen);
	}
	else {
		pOldBrush = pDC->SelectObject(&brushRed);
	}

	pDC->Rectangle(rect.Width() - 20, rect.Height() - 20, rect.Width(), rect.Height());

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
	r_pos.valid = TRUE;
	robot_pos = ConvertGridCoordToGLCoord(r_pos);
	robot_path.push_back(robot_pos);
	robot_path_data.push_back(r_pos);
	Invalidate();
	return 0;
}

LRESULT CRoboconCtrlView::reset_coord(WPARAM w, LPARAM l) {
	robot_pos.valid = FALSE;
	robot_path.clear();

	bool exist = TRUE;
	int i = 1;

	TCHAR path[MAX_PATH] = { 0 };
	GetModuleFileName(NULL, path, MAX_PATH);
	PathRemoveFileSpec(path);

	SetCurrentDirectory(path);

	while (exist) {
		std::basic_ostringstream<TCHAR> oss;
		oss << _T("data") << i <<_T(".csv");
		if (GetFileAttributes(oss.str().c_str()) == INVALID_FILE_ATTRIBUTES) {
			if (!robot_path_data.empty()) {
				std::stringstream file_path;
				file_path << "data" << i << ".csv";

				std::ofstream output_data;
				output_data.open(file_path.str());
				for (size_t i = 0; i < robot_path_data.size(); ++i) {
					output_data << robot_path_data[i].x << ", " << robot_path_data[i].y << std::endl;
				}
				output_data.close();
			}
			exist = FALSE;
		}
		else {
			++i;
		}
	}

	robot_path_data.clear();
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