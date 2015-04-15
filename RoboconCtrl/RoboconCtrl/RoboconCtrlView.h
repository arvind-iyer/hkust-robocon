
// RoboconCtrlView.h : interface of the CRoboconCtrlView class
//

#include <deque>
#include <vector>
#include <chrono>

#pragma once

class CRoboconCtrlView : public CView
{
protected: // create from serialization only
	CRoboconCtrlView();
	DECLARE_DYNCREATE(CRoboconCtrlView)

// Attributes
public:
	CRoboconCtrlDoc* GetDocument() const;

// OpenGL operations
protected:
	void GLDrawScene();
	int gl_width;
	int gl_height;

// Device and Rendering context storing
public:
	HGLRC m_hRC; // Rendering context
	HDC m_hDC; // Device context
// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	virtual int OnCreate(LPCREATESTRUCT lpCreateStruct);
	virtual void OnDestroy();
	virtual void set_selection_point(CPoint point);

// Private helper functions
private:
	struct GLCoord {
		float x;
		float y;
		unsigned int angle;
		BOOL valid;
	};

	struct GridCoord {
		int x;
		int y;
		unsigned int angle;
		std::chrono::duration<double> elapsed_time;
		BOOL valid;
	};

	struct GLColor {
		float r;
		float g;
		float b;
	};

	GLCoord GetGLCoord(CPoint wndCoord);
	GridCoord GetRobotCoord(CPoint wndCoord, unsigned int angle = 0);
	GLCoord ConvertGridCoordToGLCoord(GridCoord g);

	void DrawIndicator(GLCoord coordinates, GLColor point_ind_color, GLColor back_ind_color, GLColor point_color, int mode = 0);
	void DrawShuttleIndicator(GLCoord coordinates);

	// Positions for OpenGL
	GLCoord current_pos;
	GLCoord cursor_pos;
	GLCoord robot_pos;
	GLCoord shuttlecock_pos;

	// Mouse position on the virtual grid
	GridCoord grid_pos;

	// Selector position on the virtual grid
	GridCoord selected_grid_pos;

	// Stores path of robot for OpenGL
	std::deque<GLCoord> robot_path;

	std::vector<std::pair<GridCoord, GridCoord>> robot_path_data;

	bool xbox_status;

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
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg LRESULT refresh_coordinates(WPARAM w, LPARAM l);
	afx_msg LRESULT refresh_shuttle_coordinates(WPARAM w, LPARAM l);
	afx_msg BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg LRESULT reset_coord(WPARAM w, LPARAM l);
	afx_msg LRESULT refresh_xbox_stat(WPARAM w, LPARAM l);

	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in RoboconCtrlView.cpp
inline CRoboconCtrlDoc* CRoboconCtrlView::GetDocument() const
   { return reinterpret_cast<CRoboconCtrlDoc*>(m_pDocument); }
#endif

