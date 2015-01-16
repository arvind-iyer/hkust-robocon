
#include "stdafx.h"

#include "PropertiesWnd.h"
#include "Resource.h"
#include "MainFrm.h"
#include "RoboconCtrl.h"

#include <sstream>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

/////////////////////////////////////////////////////////////////////////////
// CResourceViewBar

CPropertiesWnd::CPropertiesWnd()
{
	m_nComboHeight = 0;
}

CPropertiesWnd::~CPropertiesWnd()
{
}

BEGIN_MESSAGE_MAP(CPropertiesWnd, CDockablePane)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_CBN_SELENDOK(ID_COMBO_BOX, OnProperties1)
	ON_COMMAND(ID_EXPAND_ALL, OnExpandAllProperties)
	ON_UPDATE_COMMAND_UI(ID_EXPAND_ALL, OnUpdateExpandAllProperties)
	ON_COMMAND(ID_SORTPROPERTIES, OnSortProperties)
	ON_UPDATE_COMMAND_UI(ID_SORTPROPERTIES, OnUpdateSortProperties)
	ON_COMMAND(ID_PROPERTIES1, OnProperties1)
	ON_UPDATE_COMMAND_UI(ID_PROPERTIES1, OnUpdateProperties1)
	ON_COMMAND(ID_PROPERTIES2, OnProperties2)
	ON_UPDATE_COMMAND_UI(ID_PROPERTIES2, OnUpdateProperties2)
	ON_WM_CONTEXTMENU()
	ON_WM_SETFOCUS()
	ON_WM_SETTINGCHANGE()
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CResourceViewBar message handlers

void CPropertiesWnd::AdjustLayout()
{
	if (GetSafeHwnd () == NULL || (AfxGetMainWnd() != NULL && AfxGetMainWnd()->IsIconic()))
	{
		return;
	}

	CRect rectClient;
	GetClientRect(rectClient);

	int cyTlb = m_wndToolBar.CalcFixedLayout(FALSE, TRUE).cy;

	m_wndObjectCombo.SetWindowPos(NULL, rectClient.left, rectClient.top, rectClient.Width(), m_nComboHeight, SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndToolBar.SetWindowPos(NULL, rectClient.left, rectClient.top + m_nComboHeight, rectClient.Width(), cyTlb, SWP_NOACTIVATE | SWP_NOZORDER);
	m_wndPropList.SetWindowPos(NULL, rectClient.left, rectClient.top + m_nComboHeight + cyTlb, rectClient.Width(), rectClient.Height() -(m_nComboHeight+cyTlb), SWP_NOACTIVATE | SWP_NOZORDER);
}

int CPropertiesWnd::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDockablePane::OnCreate(lpCreateStruct) == -1)
		return -1;

	CRect rectDummy;
	rectDummy.SetRectEmpty();

	// Create combo:
	const DWORD dwViewStyle = WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_BORDER | CBS_SORT | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

	if (!m_wndObjectCombo.Create(dwViewStyle, rectDummy, this, ID_COMBO_BOX))
	{
		TRACE0("Failed to create Properties Combo \n");
		return -1;      // fail to create
	}

	m_wndObjectCombo.AddString(_T("Application Settings"));
	m_wndObjectCombo.AddString(_T("Serial Settings"));
	m_wndObjectCombo.SetCurSel(1);

	CRect rectCombo;
	m_wndObjectCombo.GetClientRect (&rectCombo);

	m_nComboHeight = rectCombo.Height();

	if (!m_wndPropList.Create(WS_VISIBLE | WS_CHILD, rectDummy, this, 2))
	{
		TRACE0("Failed to create Properties Grid \n");
		return -1;      // fail to create
	}



	InitPropList();

	m_wndToolBar.Create(this, AFX_DEFAULT_TOOLBAR_STYLE, IDR_PROPERTIES);
	m_wndToolBar.LoadToolBar(IDR_PROPERTIES, 0, 0, TRUE /* Is locked */);
	m_wndToolBar.CleanUpLockedImages();
	m_wndToolBar.LoadBitmap(theApp.m_bHiColorIcons ? IDB_PROPERTIES_HC : IDR_PROPERTIES, 0, 0, TRUE /* Locked */);

	m_wndToolBar.SetPaneStyle(m_wndToolBar.GetPaneStyle() | CBRS_TOOLTIPS | CBRS_FLYBY);
	m_wndToolBar.SetPaneStyle(m_wndToolBar.GetPaneStyle() & ~(CBRS_GRIPPER | CBRS_SIZE_DYNAMIC | CBRS_BORDER_TOP | CBRS_BORDER_BOTTOM | CBRS_BORDER_LEFT | CBRS_BORDER_RIGHT));
	m_wndToolBar.SetOwner(this);

	// All commands will be routed via this control , not via the parent frame:
	m_wndToolBar.SetRouteCommandsViaFrame(FALSE);

	AdjustLayout();
	return 0;
}

void CPropertiesWnd::OnSize(UINT nType, int cx, int cy)
{
	CDockablePane::OnSize(nType, cx, cy);
	AdjustLayout();
}

void CPropertiesWnd::OnExpandAllProperties()
{
	m_wndPropList.ExpandAll();
}

void CPropertiesWnd::OnUpdateExpandAllProperties(CCmdUI* /* pCmdUI */)
{
}

void CPropertiesWnd::OnSortProperties()
{
	m_wndPropList.SetAlphabeticMode(!m_wndPropList.IsAlphabeticMode());
}

void CPropertiesWnd::OnUpdateSortProperties(CCmdUI* pCmdUI)
{
	pCmdUI->SetCheck(m_wndPropList.IsAlphabeticMode());
}

void CPropertiesWnd::OnProperties1()
{
	// TODO: Add your command handler code here
}

void CPropertiesWnd::OnUpdateProperties1(CCmdUI* /*pCmdUI*/)
{
	// TODO: Add your command update UI handler code here
}

void CPropertiesWnd::OnProperties2()
{
	// TODO: Add your command handler code here
}

void CPropertiesWnd::OnUpdateProperties2(CCmdUI* /*pCmdUI*/)
{
	// TODO: Add your command update UI handler code here
}

void CPropertiesWnd::OnContextMenu(CWnd* pWnd, CPoint point)
{

}

template <typename OStreamable>
std::basic_string<TCHAR> to_string(const OStreamable& o)
{
	std::basic_ostringstream<TCHAR> oss;
	oss << o;
	if (!oss) OutputDebugString(_T("ERROR: cannot convert object!"));
	return oss.str();
}

std::vector<std::basic_string<TCHAR>> CPropertiesWnd::GetSettings()
{
	std::vector < std::basic_string<TCHAR> > string_vector;
	string_vector.push_back(((CString)m_wndPropList.GetProperty(0)->GetSubItem(0)->GetValue()).GetString());
	string_vector.push_back(((CString)m_wndPropList.GetProperty(0)->GetSubItem(1)->GetValue()).GetString());
	string_vector.push_back(((CString)m_wndPropList.GetProperty(1)->GetSubItem(0)->GetValue()).GetString());
	string_vector.push_back(((CString)m_wndPropList.GetProperty(1)->GetSubItem(1)->GetValue()).GetString());
	string_vector.push_back(((CString)m_wndPropList.GetProperty(1)->GetSubItem(2)->GetValue()).GetString());
	string_vector.push_back(((CString)m_wndPropList.GetProperty(1)->GetSubItem(3)->GetValue()).GetString());

	return string_vector;
}

void CPropertiesWnd::InitPropList()
{
	SetPropListFont();

	m_wndPropList.EnableHeaderCtrl(FALSE);
	m_wndPropList.EnableDescriptionArea();
	m_wndPropList.SetVSDotNetLook();
	m_wndPropList.MarkModifiedProperties();

	CMFCPropertyGridProperty* pGroup1 = new CMFCPropertyGridProperty(_T("Communication Settings"));

//	pGroup1->AddSubItem(new CMFCPropertyGridProperty(_T("3D Look"), (_variant_t) false, _T("Specifies the window's font will be non-bold and controls will have a 3D border")));

	CMFCPropertyGridProperty* pProp = new CMFCPropertyGridProperty(_T("Port Number"), _T("COM30"), _T("Configure the port number of your serial port."));
	for (int i = 1; i < 256; i++) {
		pProp->AddOption((_T("COM") + to_string(i)).c_str());
	}
	pGroup1->AddSubItem(pProp);

	pProp = new CMFCPropertyGridProperty(_T("Baud Rate"), (_variant_t)to_string(CBR_115200).c_str(), _T("Specifies the baud rate of the connection."));
	
	// List All Microsoft Defined Baud Rates
	pProp->AddOption(to_string(CBR_110).c_str());
	pProp->AddOption(to_string(CBR_300).c_str());
	pProp->AddOption(to_string(CBR_600).c_str());
	pProp->AddOption(to_string(CBR_1200).c_str());
	pProp->AddOption(to_string(CBR_2400).c_str());
	pProp->AddOption(to_string(CBR_4800).c_str());
	pProp->AddOption(to_string(CBR_9600).c_str());
	pProp->AddOption(to_string(CBR_14400).c_str());
	pProp->AddOption(to_string(CBR_19200).c_str());
	pProp->AddOption(to_string(CBR_38400).c_str());
	pProp->AddOption(to_string(CBR_57600).c_str());
	pProp->AddOption(to_string(CBR_115200).c_str());
	pProp->AddOption(to_string(CBR_128000).c_str());
	pProp->AddOption(to_string(CBR_256000).c_str());

	pGroup1->AddSubItem(pProp);

	m_wndPropList.AddProperty(pGroup1);

	CMFCPropertyGridProperty* pGroup2 = new CMFCPropertyGridProperty(_T("Misc"));
	pProp = new CMFCPropertyGridProperty(_T("Buffer Size"), (_variant_t)to_string(500).c_str(), _T("Specifies the size of the read buffer."));
	pGroup2->AddSubItem(pProp);

	pProp = new CMFCPropertyGridProperty(_T("Write Mode"), (_variant_t)to_string(2).c_str(), _T("0 - Enter to send\n1 - Realtime sending\n2 - Realtime sending everywhere"));
	pGroup2->AddSubItem(pProp);

	pProp = new CMFCPropertyGridProperty(_T("Pad Bits"), (_variant_t)to_string(1).c_str(), _T("0 - Disable\n1 - Enable"));
	pGroup2->AddSubItem(pProp);

	pProp = new CMFCPropertyGridProperty(_T("Read Mode"), (_variant_t)to_string(1).c_str(), _T("0 - Regular Read\n1 - Robot coordinates"));
	pGroup2->AddSubItem(pProp);

	m_wndPropList.AddProperty(pGroup2);
}

void CPropertiesWnd::OnSetFocus(CWnd* pOldWnd)
{
	CDockablePane::OnSetFocus(pOldWnd);
	m_wndPropList.SetFocus();
}

void CPropertiesWnd::OnSettingChange(UINT uFlags, LPCTSTR lpszSection)
{
	CDockablePane::OnSettingChange(uFlags, lpszSection);
	SetPropListFont();
}

void CPropertiesWnd::SetPropListFont()
{
	::DeleteObject(m_fntPropList.Detach());

	LOGFONT lf;
	afxGlobalData.fontRegular.GetLogFont(&lf);

	NONCLIENTMETRICS info;
	info.cbSize = sizeof(info);

	afxGlobalData.GetNonClientMetrics(info);

	lf.lfHeight = info.lfMenuFont.lfHeight;
	lf.lfWeight = info.lfMenuFont.lfWeight;
	lf.lfItalic = info.lfMenuFont.lfItalic;

	m_fntPropList.CreateFontIndirect(&lf);

	m_wndPropList.SetFont(&m_fntPropList);
	m_wndObjectCombo.SetFont(&m_fntPropList);
}
