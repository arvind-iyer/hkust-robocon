
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

// Private helper functions
private:
	struct GLCoord {
		float x;
		float y;
		BOOL valid;
	};

	GLCoord GetGLCoord(CPoint wndCoord);
	std::pair<CPoint, BOOL> GetRobotCoord();
	struct GLCoord current_pos;
	struct GLCoord cursor_pos;

	int robot_x_pos;
	int robot_y_pos;
	unsigned int robot_angle;
	BOOL robot_pos_valid;

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
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg LRESULT refresh_coordinates(WPARAM w, LPARAM l);
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in RoboconCtrlView.cpp
inline CRoboconCtrlDoc* CRoboconCtrlView::GetDocument() const
   { return reinterpret_cast<CRoboconCtrlDoc*>(m_pDocument); }
#endif

