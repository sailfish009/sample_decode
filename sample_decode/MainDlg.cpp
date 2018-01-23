#include "stdafx.h"
#include "MainDlg.h"
#include <atldlgs.h>

std::atomic<UINT8> video_off =  FALSE;
BOOL            CMainDlg::toggle_video = FALSE;
HWND          CMainDlg::view_hwnd = nullptr;

AVCodecContext       *avc_ctx = nullptr;
AVFormatContext     *avf_ctx = nullptr;
AVPacket packet;
int video_stream_index = -1;

// video asio
boost::asio::io_service video_io_service;
boost::asio::io_service::work video_work(video_io_service);
boost::thread_group video_threads;
boost::asio::io_service::strand video_strand(video_io_service);

void video_stream(const UINT8& channel, boost::asio::mutable_buffers_1 v);

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

  video_threads.create_thread(boost::bind(&boost::asio::io_service::run, &video_io_service));

  // HW decoder run 
  /////////////////////////
  /////////////////////////
  av_register_all();
  avformat_network_init();
  avf_ctx = avformat_alloc_context();

  std::string str = "rtsp://admin:12345@192.168.100.100/Streaming/Channels/101";

  if (avformat_open_input(&avf_ctx, str.c_str(), nullptr, nullptr) != 0)
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

  av_init_packet(&packet);

  stream_work(TRUE);

  toggle_video = TRUE;
  return TRUE;
}

BOOL CMainDlg::stop_video()
{
  stream_work(FALSE);

  video_io_service.stop();
  video_threads.join_all();

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
    if ( (av_read_frame(avf_ctx, &packet) >= 0) && (packet.stream_index == video_stream_index) )
    {
      video_strand.post(boost::bind(video_stream, 0, boost::asio::buffer(packet.buf->data ,  packet.buf->size )));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}


void video_stream(const UINT8& channel, boost::asio::mutable_buffers_1 v)
{
  std::size_t buffer_size = boost::asio::buffer_size(v);
  unsigned char* buffer = boost::asio::buffer_cast<unsigned char*>(v);
  dummy_hw_decoder_function(0, buffer, buffer_size);
}