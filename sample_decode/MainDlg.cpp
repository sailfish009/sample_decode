//========================================================================
// sample_decode - rtsp packet to hw decoder (hw sdk omitted)
//------------------------------------------------------------------------
// Copyright (c) 2017-2018 Ji Wong Park <sailfish009@gmail.com>
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would
//    be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not
//    be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source
//    distribution.
//
//========================================================================

#include "stdafx.h"
#include "MainDlg.h"
#include <atldlgs.h>

std::atomic<UINT8> video_off =  FALSE;
BOOL            CMainDlg::toggle_video = FALSE;
HWND          CMainDlg::view_hwnd = nullptr;

AVCodecContext       *avc_ctx = nullptr;
AVFormatContext     *avf_ctx = nullptr;
int video_stream_index = -1;

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

  // HW decoder run 
  /////////////////////////
  /////////////////////////

  av_register_all();
  avformat_network_init();
  avf_ctx = avformat_alloc_context();

  AVDictionary* option = nullptr;
  av_dict_set(&option, "rtsp_transport", "tcp", 0);

  std::string str = "rtsp://admin:12345@192.168.100.100/Streaming/Channels/101";

  if (avformat_open_input(&avf_ctx, str.c_str(), nullptr, &option) != 0)
  {
    printf("avformat_open_input error\n");
    return FALSE;
  }

  if (avformat_find_stream_info(avf_ctx,  nullptr) <0 )
  {
    avformat_close_input(&avf_ctx);
    printf("avformat_find_stream_info error\n");
    return FALSE;
  }

  for (int i = 0; i < avf_ctx->nb_streams; i++) {
    if (avf_ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
      video_stream_index = i;
  }

  if (video_stream_index == -1)
  {
    avformat_close_input(&avf_ctx);
    printf("avformat_find_stream_info error\n");
    return FALSE;
  }

  avc_ctx = avf_ctx->streams[video_stream_index]->codec;

  AVCodec *avc = avcodec_find_decoder(avc_ctx->codec_id);
  if (avc == nullptr)
  {
    avformat_close_input(&avf_ctx);
    printf("avcodec_find_decoder error\n");
    return FALSE;
  }

  if(avcodec_open2(avc_ctx, avc, nullptr) < 0)
  {
    avcodec_close(avc_ctx);
    avformat_close_input(&avf_ctx);
    printf("avcodec_open2 error\n");
    return FALSE;
  }


  stream_work(TRUE);

  toggle_video = TRUE;
  return TRUE;
}

BOOL CMainDlg::stop_video()
{
  stream_work(FALSE);

  // HW decoder uninit
  /////////////////////////
  /////////////////////////

  toggle_video = FALSE;
  return TRUE;
}

void CMainDlg::stream_work(UINT8 onoff)
{
  switch (onoff)
  {
  case 0:   video_off = TRUE; Sleep(1000); break;
  default: video_off = FALSE; stream_worker = std::thread([=] { stream(); });  stream_worker.detach();  break;
  }
}

void CMainDlg::stream()
{
  for (;;)
  {
    switch (video_off) { case 0:break; default: return; }
    AVPacket packet;
    av_init_packet(&packet);
    if ( (av_read_frame(avf_ctx, &packet) >= 0) && (packet.stream_index == video_stream_index) )
    {
      dummy_hw_decoder_function(0, packet.buf->data, packet.buf->size);
      av_packet_unref(&packet);
    }
  }
}
