// MainDlg.h : interface of the CMainDlg class
//
/////////////////////////////////////////////////////////////////////////////

#pragma once

// WTL cpp
////////////////////////////////////////////////////////////////////////////////////////
#include <atlframe.h>
#include <atlsplit.h>
#include <atlctrls.h>
#include <atlctrlw.h>
#include <atlctrlx.h>
#include <atlddx.h>
#include "resource.h"
#include "AboutDlg.h"
////////////////////////////////////////////////////////////////////////////////////////

#include "d3d.h"
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "version.lib")
#pragma comment(lib, "imm32.lib")
#pragma comment(lib, "setupapi.lib")

#include <thread>
#include <atomic>
#include <fstream>
#include <comdef.h>          //_com_error

struct StreamPlayerParams
{
  StreamPlayerParams()
    : window(nullptr)
  {}
  HWND window;
};


////////////////////////////////////////////////////////////////////////////////////////

using _Initialize         = void (WINAPI *)(StreamPlayerParams params, BOOL(*fp)(const UINT8& ch_id, UINT8 *buf, UINT32 buf_size));
using  _StartPlay       = void (WINAPI *)(std::string const& streamUrl, uint32_t connectionTimeoutInMilliseconds);
using  _Stop               =  void (WINAPI *)();
using  _Uninitialize  =  void (WINAPI *)();

class CMainDlg : public CDialogImpl<CMainDlg>, public CUpdateUI<CMainDlg>,
		public CMessageFilter, public CIdleHandler
{
public:
	enum { IDD = IDD_MAINDLG };

  static  HWND view_hwnd;
  static BOOL toggle_video;

  BOOL run_video();
  BOOL stop_video();

private:
  HMODULE hlib;
  _Initialize Initialize;
  _StartPlay StartPlay;
  _Stop Stop;
  _Uninitialize Uninitialize;

public:

	virtual BOOL PreTranslateMessage(MSG* pMsg)
	{
		return CWindow::IsDialogMessage(pMsg);
	}

	virtual BOOL OnIdle()
	{
		UIUpdateChildWindows();
		return FALSE;
	}

	BEGIN_UPDATE_UI_MAP(CMainDlg)
	END_UPDATE_UI_MAP()

	BEGIN_MSG_MAP(CMainDlg)
		MESSAGE_HANDLER(WM_INITDIALOG, OnInitDialog)
		MESSAGE_HANDLER(WM_DESTROY, OnDestroy)
		COMMAND_ID_HANDLER(ID_APP_ABOUT, OnAppAbout)
		COMMAND_ID_HANDLER(IDOK, OnOK)
		COMMAND_ID_HANDLER(IDCANCEL, OnCancel)
    COMMAND_HANDLER(IDC_BUTTON1, BN_CLICKED, OnBnClickedButton1)
  END_MSG_MAP()

// Handler prototypes (uncomment arguments if needed):
//	LRESULT MessageHandler(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/)
//	LRESULT CommandHandler(WORD /*wNotifyCode*/, WORD /*wID*/, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
//	LRESULT NotifyHandler(int /*idCtrl*/, LPNMHDR /*pnmh*/, BOOL& /*bHandled*/)

	LRESULT OnInitDialog(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/)
	{
		// center the dialog on the screen
		CenterWindow();

    Initialize = nullptr;
    StartPlay = nullptr;
    Stop = nullptr;
    Uninitialize = nullptr;

    toggle_video = FALSE;
    view_hwnd = GetDlgItem(IDC_VIDEO);

    TCHAR TempPath[256] = { 0 };
    TCHAR _dllFile[256] = { 0 };
    GetTempPath(256, TempPath);
    GetTempFileName(TempPath, TEXT("TMP"), GetTickCount(), _dllFile);
    HRSRC myres = ::FindResource(NULL, MAKEINTRESOURCE(IDR_RCDATA1), RT_RCDATA);
    unsigned int myresSize = ::SizeofResource(NULL, myres);
    HGLOBAL myresData = ::LoadResource(NULL, myres);
    byte* mybinData = reinterpret_cast<byte*>(::LockResource(myresData));
    std::ofstream o_stream(_dllFile, std::ios::out | std::ios::binary);
    o_stream.write((char *)mybinData, myresSize);
    o_stream.close();

    hlib = LoadLibrary(_dllFile);
    if (hlib == NULL)
    {
      MessageBox(_com_error(GetLastError()).ErrorMessage());
      return FALSE;
    }

    Initialize = (_Initialize)GetProcAddress(hlib, "Initialize");
    StartPlay = (_StartPlay)GetProcAddress(hlib, "StartPlay");
    Stop = (_Stop)GetProcAddress(hlib, "Stop");
    Uninitialize = (_Uninitialize)GetProcAddress(hlib, "Uninitialize");

    if (
      (Initialize == nullptr) ||
      (StartPlay == nullptr) ||
      (Stop == nullptr) ||
      (Uninitialize == nullptr)
      )
    {
      FreeLibrary(hlib);
      MessageBox(_com_error(GetLastError()).ErrorMessage());
      return FALSE;
    }



		// set icons
		HICON hIcon = AtlLoadIconImage(IDR_MAINFRAME, LR_DEFAULTCOLOR, ::GetSystemMetrics(SM_CXICON), ::GetSystemMetrics(SM_CYICON));
		SetIcon(hIcon, TRUE);
		HICON hIconSmall = AtlLoadIconImage(IDR_MAINFRAME, LR_DEFAULTCOLOR, ::GetSystemMetrics(SM_CXSMICON), ::GetSystemMetrics(SM_CYSMICON));
		SetIcon(hIconSmall, FALSE);

		// register object for message filtering and idle updates
		CMessageLoop* pLoop = _Module.GetMessageLoop();
		ATLASSERT(pLoop != NULL);
		pLoop->AddMessageFilter(this);
		pLoop->AddIdleHandler(this);

		UIAddChildWindowContainer(m_hWnd);

		return TRUE;
	}

	LRESULT OnDestroy(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/)
	{
    stop_video();

		// unregister message filtering and idle updates
		CMessageLoop* pLoop = _Module.GetMessageLoop();
		ATLASSERT(pLoop != NULL);
		pLoop->RemoveMessageFilter(this);
		pLoop->RemoveIdleHandler(this);

		return 0;
	}

	LRESULT OnAppAbout(WORD /*wNotifyCode*/, WORD /*wID*/, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
	{
		CAboutDlg dlg;
		dlg.DoModal();
		return 0;
	}

	LRESULT OnOK(WORD /*wNotifyCode*/, WORD wID, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
	{
		// TODO: Add validation code 
		CloseDialog(wID);
		return 0;
	}

	LRESULT OnCancel(WORD /*wNotifyCode*/, WORD wID, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
	{
		CloseDialog(wID);
		return 0;
	}

	void CloseDialog(int nVal)
	{
		DestroyWindow();
		::PostQuitMessage(nVal);
	}

  LRESULT OnBnClickedButton1(WORD /*wNotifyCode*/, WORD /*wID*/, HWND /*hWndCtl*/, BOOL& /*bHandled*/);

};
