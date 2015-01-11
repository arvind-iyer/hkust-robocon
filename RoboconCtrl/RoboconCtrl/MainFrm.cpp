
// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "RoboconCtrl.h"

#include "MainFrm.h"
#include <sstream>
#include <locale>
#include <codecvt>
#include <string>
#include <utility>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWndEx)

const int  iMaxUserToolbars = 10;
const UINT uiFirstUserToolBarId = AFX_IDW_CONTROLBAR_FIRST + 40;
const UINT uiLastUserToolBarId = uiFirstUserToolBarId + iMaxUserToolbars - 1;

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWndEx)
	ON_WM_CREATE()
	ON_COMMAND(ID_VIEW_CUSTOMIZE, &CMainFrame::OnViewCustomize)
	ON_UPDATE_COMMAND_UI(ID_FILE_SAVE, &CMainFrame::OnUpdateFileSave)
	ON_UPDATE_COMMAND_UI(ID_FILE_NEW, &CMainFrame::OnUpdateFileNew)
	ON_COMMAND(ID_EDIT_COPY, &CMainFrame::OnEditCopy)
	ON_COMMAND(ID_EDIT_CLEAR, &CMainFrame::OnEditClear)
	ON_COMMAND(ID_EDIT_PASTE, &CMainFrame::OnEditPaste)
	ON_COMMAND(ID_EDIT_SELECT_ALL, &CMainFrame::OnEditSelectAll)
	ON_COMMAND(ID_FILE_OPEN, &CMainFrame::OpenConnection)
	ON_COMMAND(ID_FILE_UPDATE, &CMainFrame::CloseConnection)
	ON_REGISTERED_MESSAGE(AFX_WM_CREATETOOLBAR, &CMainFrame::OnToolbarCreateNew)
	ON_MESSAGE(WM_SEND_STRING, &CMainFrame::WriteString)
	ON_WM_SETTINGCHANGE()
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // status line indicator
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};

// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{
	// TODO: add member initialization code here
}

SerialIO* CMainFrame::serial = NULL;

CMainFrame::~CMainFrame()
{
	if (serial) {
		delete serial;
		serial = NULL;
	}
}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFrameWndEx::OnCreate(lpCreateStruct) == -1)
		return -1;

	BOOL bNameValid;

	if (!m_wndMenuBar.Create(this))
	{
		TRACE0("Failed to create menubar\n");
		return -1;      // fail to create
	}

	m_wndMenuBar.SetPaneStyle(m_wndMenuBar.GetPaneStyle() | CBRS_SIZE_DYNAMIC | CBRS_TOOLTIPS | CBRS_FLYBY);

	// prevent the menu bar from taking the focus on activation
	CMFCPopupMenu::SetForceMenuFocus(FALSE);

	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP | CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(theApp.m_bHiColorIcons ? IDR_MAINFRAME_256 : IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}

	CString strToolBarName;
	bNameValid = strToolBarName.LoadString(IDS_TOOLBAR_STANDARD);
	ASSERT(bNameValid);
	m_wndToolBar.SetWindowText(strToolBarName);

	CString strCustomize;
	bNameValid = strCustomize.LoadString(IDS_TOOLBAR_CUSTOMIZE);
	ASSERT(bNameValid);
	m_wndToolBar.EnableCustomizeButton(TRUE, ID_VIEW_CUSTOMIZE, strCustomize);

	// Allow user-defined toolbars operations:
	InitUserToolbars(NULL, uiFirstUserToolBarId, uiLastUserToolBarId);

	if (!m_wndStatusBar.Create(this))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}
	m_wndStatusBar.SetIndicators(indicators, sizeof(indicators)/sizeof(UINT));

	// TODO: Delete these five lines if you don't want the toolbar and menubar to be dockable
	m_wndMenuBar.EnableDocking(CBRS_ALIGN_ANY);
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockPane(&m_wndMenuBar);
	DockPane(&m_wndToolBar);


	// enable Visual Studio 2005 style docking window behavior
	CDockingManager::SetDockingMode(DT_SMART);
	// enable Visual Studio 2005 style docking window auto-hide behavior
	EnableAutoHidePanes(CBRS_ALIGN_ANY);

	// create docking windows
	if (!CreateDockingWindows())
	{
		TRACE0("Failed to create docking windows\n");
		return -1;
	}

	m_wndOutput.EnableDocking(CBRS_ALIGN_ANY);
	DockPane(&m_wndOutput);
	m_wndProperties.EnableDocking(CBRS_ALIGN_ANY);
	DockPane(&m_wndProperties);
	m_wndInput.EnableDocking(CBRS_ALIGN_ANY);
	DockPane(&m_wndInput);


	// set the visual manager used to draw all user interface elements
	CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerVS2008));

	// Enable toolbar and docking window menu replacement
	EnablePaneMenu(TRUE, ID_VIEW_CUSTOMIZE, strCustomize, ID_VIEW_TOOLBAR);

	// enable quick (Alt+drag) toolbar customization
	CMFCToolBar::EnableQuickCustomization();

	if (CMFCToolBar::GetUserImages() == NULL)
	{
		// load user-defined toolbar images
		if (m_UserImages.Load(_T(".\\UserImages.bmp")))
		{
			CMFCToolBar::SetUserImages(&m_UserImages);
		}
	}

	// enable menu personalization (most-recently used commands)
	// TODO: define your own basic commands, ensuring that each pulldown menu has at least one basic command.
	CList<UINT, UINT> lstBasicCommands;

	lstBasicCommands.AddTail(ID_FILE_OPEN);
	lstBasicCommands.AddTail(ID_FILE_UPDATE);
	lstBasicCommands.AddTail(ID_APP_EXIT);
	lstBasicCommands.AddTail(ID_APP_ABOUT);
	lstBasicCommands.AddTail(ID_VIEW_STATUS_BAR);
	lstBasicCommands.AddTail(ID_VIEW_TOOLBAR);

	CMFCToolBar::SetBasicCommands(lstBasicCommands);

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWndEx::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return TRUE;
}

BOOL CMainFrame::CreateDockingWindows()
{
	BOOL bNameValid;
	// Create output window
	CString strOutputWnd;
	bNameValid = strOutputWnd.LoadString(IDS_OUTPUT_WND);
	ASSERT(bNameValid);
	if (!m_wndOutput.Create(strOutputWnd, this, CRect(0, 0, 100, 100), TRUE, ID_VIEW_OUTPUTWND, WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_BOTTOM | CBRS_FLOAT_MULTI))
	{
		TRACE0("Failed to create Output window\n");
		return FALSE; // failed to create
	}

	// Create properties window
	CString strPropertiesWnd;
	bNameValid = strPropertiesWnd.LoadString(IDS_PROPERTIES_WND);
	ASSERT(bNameValid);
	if (!m_wndProperties.Create(strPropertiesWnd, this, CRect(0, 0, 200, 200), TRUE, ID_VIEW_PROPERTIESWND, WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_RIGHT | CBRS_FLOAT_MULTI))
	{
		TRACE0("Failed to create Properties window\n");
		return FALSE; // failed to create
	}

	// Create input window
	CString strInputWnd;
	bNameValid = strInputWnd.LoadString(IDS_INPUT_WND);
	ASSERT(bNameValid);
	if (!m_wndInput.Create(strInputWnd, this, CRect(0, 0, 100, 100), TRUE, ID_VIEW_INPUTWND, WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN | CBRS_BOTTOM | CBRS_FLOAT_MULTI))
	{
		TRACE0("Failed to create Output window\n");
		return FALSE; // failed to create
	}

	SetDockingWindowIcons(theApp.m_bHiColorIcons);
	return TRUE;
}

void CMainFrame::SetDockingWindowIcons(BOOL bHiColorIcons)
{
	HICON hOutputBarIcon = (HICON) ::LoadImage(::AfxGetResourceHandle(), MAKEINTRESOURCE(bHiColorIcons ? IDI_OUTPUT_WND_HC : IDI_OUTPUT_WND), IMAGE_ICON, ::GetSystemMetrics(SM_CXSMICON), ::GetSystemMetrics(SM_CYSMICON), 0);
	m_wndOutput.SetIcon(hOutputBarIcon, FALSE);

	HICON hPropertiesBarIcon = (HICON) ::LoadImage(::AfxGetResourceHandle(), MAKEINTRESOURCE(bHiColorIcons ? IDI_PROPERTIES_WND_HC : IDI_PROPERTIES_WND), IMAGE_ICON, ::GetSystemMetrics(SM_CXSMICON), ::GetSystemMetrics(SM_CYSMICON), 0);
	m_wndProperties.SetIcon(hPropertiesBarIcon, FALSE);

	HICON hInputBarIcon = (HICON) ::LoadImage(::AfxGetResourceHandle(), MAKEINTRESOURCE(bHiColorIcons ? IDI_INPUT_WND_HC : IDI_INPUT_WND), IMAGE_ICON, ::GetSystemMetrics(SM_CXSMICON), ::GetSystemMetrics(SM_CYSMICON), 0);
	m_wndInput.SetIcon(hInputBarIcon, FALSE);
}

void CMainFrame::OnUpdateFileNew(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(FALSE);
}
void CMainFrame::OnUpdateFileSave(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(FALSE);
}

// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWndEx::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWndEx::Dump(dc);
}
#endif //_DEBUG

// Settings Getters (I have no idea how to make this better)

std::vector<std::basic_string<TCHAR>> CMainFrame::GetSettings()
{
	return m_wndProperties.GetSettings();
}

// Passing print functions to output window

void CMainFrame::print_to_output(std::basic_string<TCHAR> string_to_print)
{
	m_wndOutput.PrintString(string_to_print);
}
void CMainFrame::print_from_serial(std::basic_string<TCHAR> string_to_print)
{
	m_wndOutput.ReadFromSerial(string_to_print);
}

// CMainFrame threads

// Read thread

void __cdecl CMainFrame::read_thread(void* app_ptr){

	while (serial != NULL && serial->is_connected()){
		if (serial->bytes_to_read()) {
			std::string read_string = serial->read();
			std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
			((CMainFrame*)AfxGetMainWnd())->print_from_serial(converter.from_bytes(read_string));
		}
		Sleep(10);
	}
	_endthread();
}

// CMainFrame message handlers

void CMainFrame::OpenConnection()
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

	if (serial != NULL){
		CloseConnection();
	}
	std::vector<std::basic_string<TCHAR>> settings = m_wndProperties.GetSettings();
	print_to_output(_T("Opening Port: ") + settings[0] + _T(" | Baud rate: ") + settings[1] + _T(" bit/s | Read buffer size: ") + settings[2] + _T(" bytes"));
	try {
		serial = new SerialIO(converter.to_bytes(settings[0]), stoi(settings[1]), stoi(settings[2]));

		if (serial->is_connected()) {
			print_to_output(_T("SUCCESS!"));
		}
		_beginthread(CMainFrame::read_thread, 0, this);
	}
	catch (std::runtime_error r) {
		print_to_output(converter.from_bytes(r.what()));
	}
}

void CMainFrame::CloseConnection()
{
	if (serial) {
		print_to_output(_T("Closing Port"));
		delete serial;
		serial = NULL;
	}
}

void CMainFrame::OnEditCopy()
{
	CWnd* pCVw = GetFocus();
	if (pCVw != NULL && pCVw != this) {
		if (!(pCVw->PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_COPY, 0), 0))) {
//			std::basic_ostringstream<TCHAR> oss;
//			oss << _T("ID of Window: ") << pCVw->GetDlgCtrlID();
//			OutputDebugString(oss.str().c_str());
			OutputDebugString(_T("ERROR: cannot post copy message!"));
		}
	}
}

void CMainFrame::OnEditClear()
{
	if (!m_wndInput.m_wndInputBox.PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_CLEAR, 0), 0)) {
		OutputDebugString(_T("ERROR: cannot post clear message to input window!"));
	}
	if (!m_wndOutput.m_wndOutputBuild.PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_CLEAR, 0), 0)) {
		OutputDebugString(_T("ERROR: cannot post clear message to output window!"));
	}
	if (!m_wndOutput.m_wndOutputRead.PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_CLEAR, 0), 0)) {
		OutputDebugString(_T("ERROR: cannot post clear message to output window!"));
	}
}

void CMainFrame::OnEditPaste()
{
	CWnd* pCVw = GetFocus();
	if (pCVw != NULL && pCVw != this) {
		if (!(pCVw->PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_PASTE, 0), 0))) {
			OutputDebugString(_T("ERROR: cannot post paste message!"));
		}
	}
}

void CMainFrame::OnEditSelectAll()
{
	CWnd* pCVw = GetFocus();
	if (pCVw != NULL && pCVw != this) {
		if (!(pCVw->PostMessage(WM_COMMAND, MAKEWPARAM(ID_EDIT_SELECT_ALL, 0), 0))) {
			OutputDebugString(_T("ERROR: cannot post select all message!"));
		}
	}
}

LRESULT CMainFrame::WriteString(WPARAM w, LPARAM l)
{
	if (serial == NULL) {
		OpenConnection();
	}
	if (serial != NULL && serial->is_connected()){
		CString* string = reinterpret_cast<CString*>(l);
		CT2CA converted_string(*string);
		serial->write(std::string(converted_string));
	}
	else {
		OutputDebugString(_T("Port cannot be opened, cannot write"));
		return 1;
	}
	return 0;
}

void CMainFrame::OnViewCustomize()
{
	CMFCToolBarsCustomizeDialog* pDlgCust = new CMFCToolBarsCustomizeDialog(this, TRUE /* scan menus */);
	pDlgCust->EnableUserDefinedToolbars();
	pDlgCust->Create();
}

LRESULT CMainFrame::OnToolbarCreateNew(WPARAM wp,LPARAM lp)
{
	LRESULT lres = CFrameWndEx::OnToolbarCreateNew(wp,lp);
	if (lres == 0)
	{
		return 0;
	}

	CMFCToolBar* pUserToolbar = (CMFCToolBar*)lres;
	ASSERT_VALID(pUserToolbar);

	BOOL bNameValid;
	CString strCustomize;
	bNameValid = strCustomize.LoadString(IDS_TOOLBAR_CUSTOMIZE);
	ASSERT(bNameValid);

	pUserToolbar->EnableCustomizeButton(TRUE, ID_VIEW_CUSTOMIZE, strCustomize);
	return lres;
}


BOOL CMainFrame::LoadFrame(UINT nIDResource, DWORD dwDefaultStyle, CWnd* pParentWnd, CCreateContext* pContext) 
{
	// base class does the real work

	if (!CFrameWndEx::LoadFrame(nIDResource, dwDefaultStyle, pParentWnd, pContext))
	{
		return FALSE;
	}


	// enable customization button for all user toolbars
	BOOL bNameValid;
	CString strCustomize;
	bNameValid = strCustomize.LoadString(IDS_TOOLBAR_CUSTOMIZE);
	ASSERT(bNameValid);

	for (int i = 0; i < iMaxUserToolbars; i ++)
	{
		CMFCToolBar* pUserToolbar = GetUserToolBarByIndex(i);
		if (pUserToolbar != NULL)
		{
			pUserToolbar->EnableCustomizeButton(TRUE, ID_VIEW_CUSTOMIZE, strCustomize);
		}
	}

	return TRUE;
}


void CMainFrame::OnSettingChange(UINT uFlags, LPCTSTR lpszSection)
{
	CFrameWndEx::OnSettingChange(uFlags, lpszSection);
	m_wndOutput.UpdateFonts();
}
