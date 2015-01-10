
// RoboconCtrl.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "afxwinappex.h"
#include "afxdialogex.h"
#include "RoboconCtrl.h"
#include "MainFrm.h"

#include "RoboconCtrlDoc.h"
#include "RoboconCtrlView.h"

#include <locale>
#include <codecvt>
#include <string>
#include <utility>
#include <sstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CRoboconCtrlApp

BEGIN_MESSAGE_MAP(CRoboconCtrlApp, CWinAppEx)
	ON_COMMAND(ID_APP_ABOUT, &CRoboconCtrlApp::OnAppAbout)
	// Standard file based document commands
	ON_COMMAND(ID_FILE_OPEN, &CRoboconCtrlApp::OpenConnection)
	ON_COMMAND(ID_FILE_UPDATE, &CRoboconCtrlApp::CloseConnection)
END_MESSAGE_MAP()


// CRoboconCtrlApp construction

CRoboconCtrlApp::CRoboconCtrlApp()
{
	m_bHiColorIcons = TRUE;

	// TODO: replace application ID string below with unique ID string; recommended
	// format for string is CompanyName.ProductName.SubProduct.VersionInformation
	SetAppID(_T("RoboconCtrl.AppID.NoVersion"));

	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

CRoboconCtrlApp::~CRoboconCtrlApp()
{
	if (serial) {
		delete serial;
		serial = NULL;
	}
}

// The one and only CRoboconCtrlApp object

CRoboconCtrlApp theApp;

SerialIO* CRoboconCtrlApp::serial = NULL;

// CRoboconCtrlApp initialization

BOOL CRoboconCtrlApp::InitInstance()
{
	// InitCommonControlsEx() is required on Windows XP if an application
	// manifest specifies use of ComCtl32.dll version 6 or later to enable
	// visual styles.  Otherwise, any window creation will fail.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Set this to include all the common control classes you want to use
	// in your application.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinAppEx::InitInstance();


	EnableTaskbarInteraction(FALSE);

	// AfxInitRichEdit2() is required to use RichEdit control	
	// AfxInitRichEdit2();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	// of your final executable, you should remove from the following
	// the specific initialization routines you do not need
	// Change the registry key under which our settings are stored
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization
	SetRegistryKey(_T("Local AppWizard-Generated Applications"));
	LoadStdProfileSettings(0);  // Load standard INI file options (including MRU)


	InitContextMenuManager();

	InitKeyboardManager();

	InitTooltipManager();
	CMFCToolTipInfo ttParams;
	ttParams.m_bVislManagerTheme = TRUE;
	theApp.GetTooltipManager()->SetTooltipParams(AFX_TOOLTIP_TYPE_ALL,
		RUNTIME_CLASS(CMFCToolTipCtrl), &ttParams);

	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CRoboconCtrlDoc),
		RUNTIME_CLASS(CMainFrame),       // main SDI frame window
		RUNTIME_CLASS(CRoboconCtrlView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);


	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);



	// Dispatch commands specified on the command line.  Will return FALSE if
	// app was launched with /RegServer, /Register, /Unregserver or /Unregister.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// The one and only window has been initialized, so show and update it
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	return TRUE;
}

// Printing messages

void CRoboconCtrlApp::PrintOutput(std::string string, int output_number) // 0 for All logs, 1 for Serial only
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	switch (output_number) {
	case 1:
		((CMainFrame*)m_pMainWnd)->print_from_serial((converter.from_bytes(string)).c_str()); break;
	case 0:
	default:
		((CMainFrame*)m_pMainWnd)->print_to_output((converter.from_bytes(string)).c_str()); break;
	}
}

void CRoboconCtrlApp::PrintOutput(std::wstring string, int output_number) // 0 for All logs, 1 for Serial only
{
	switch (output_number) {
	case 1:
		((CMainFrame*)m_pMainWnd)->print_from_serial(string.c_str()); break;
	case 0:
	default:
		((CMainFrame*)m_pMainWnd)->print_to_output(string.c_str()); break;
	}
}

// Read thread

void __cdecl CRoboconCtrlApp::read_thread(void* app_ptr){

	while (serial != NULL && serial->is_connected()){
		if (serial->bytes_to_read() && !(serial->read()).empty()) {
			((CRoboconCtrlApp*)app_ptr)->PrintOutput(serial->read(), 1);
		}
		Sleep(10);
	}
	_endthread();
}

// CRoboconCtrlApp message handlers

void CRoboconCtrlApp::OpenConnection()
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

	if (serial != NULL){
		CloseConnection();
	}
	std::vector<std::basic_string<TCHAR>> settings = ((CMainFrame*)m_pMainWnd)->GetSettings();
	PrintOutput(_T("Opening Port: ") + settings[0] + _T(" | Baud rate: ") + settings[1] + _T(" bit/s | Read buffer size: ") + settings[2] + _T(" bytes"), 0);
	try {
		serial = new SerialIO(converter.to_bytes(settings[0]),stoi(settings[1]),stoi(settings[2]));

		if (serial->is_connected()) {
			PrintOutput("SUCCESS!", 0);
		}
		_beginthread(CRoboconCtrlApp::read_thread, 0, this);
	}
	catch (std::runtime_error r) {
		PrintOutput(r.what(), 0);
	}
}

void CRoboconCtrlApp::CloseConnection()
{
	if (serial) {
		PrintOutput("Closing Port", 0);
		delete serial;
		serial = NULL;
	}
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// App command to run the dialog
void CRoboconCtrlApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}

// CRoboconCtrlApp customization load/save methods

void CRoboconCtrlApp::PreLoadState()
{
	BOOL bNameValid;
	CString strName;
	bNameValid = strName.LoadString(IDS_EDIT_MENU);
	ASSERT(bNameValid);
	GetContextMenuManager()->AddMenu(strName, IDR_POPUP_EDIT);
}

// CRoboconCtrlApp message handlers



