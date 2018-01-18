#include "stdafx.h"
#include "MainDlg.h"
#include <atldlgs.h>

std::atomic<UINT8> video_off =  FALSE;
BOOL            CMainDlg::toggle_video = FALSE;
HWND          CMainDlg::view_hwnd = nullptr;

static BOOL  dummy_hw_decoder_function(const UINT8& channel, UINT8 *buffer, UINT32 buffer_size)
{
  // ...
  return TRUE;
}


LRESULT CMainDlg::OnBnClickedButton1(WORD /*wNotifyCode*/, WORD /*wID*/, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
{
  if (toggle_video == FALSE)
  {
    ::SetWindowText(GetDlgItem(IDC_BUTTON1), L"STOP");
    run_video();
  }
  else
  {
    ::SetWindowText(GetDlgItem(IDC_BUTTON1), L"RUN");
    stop_video();
  }

  return 0;
}

BOOL CMainDlg::run_video()
{
  // HW decoder init
  /////////////////////////
  /////////////////////////

  StreamPlayerParams params;
  params.window = view_hwnd;
  Initialize(params, &dummy_hw_decoder_function);

  // HW decoder run 
  /////////////////////////
  /////////////////////////

  std::string str = "rtsp://admin:12345@192.168.100.100/Streaming/Channels/101";
  StartPlay(str, 10000);

  toggle_video = TRUE;
  return FALSE;
}

BOOL CMainDlg::stop_video()
{
  // ffmpeg rtsp stop
  Stop();

  video_off = TRUE;
  Sleep(100);

  // HW decoder uninit
  /////////////////////////
  /////////////////////////

  toggle_video = FALSE;
  return TRUE;
}

