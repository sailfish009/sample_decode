#include "stream.h"
#include <stdexcept>

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace FFmpeg;
using namespace FFmpeg::Facade;

Stream::Stream(string const& streamUrl, int32_t connectionTimeoutInMilliseconds)
    : connectionTimeout_(connectionTimeoutInMilliseconds), stopRequested_(false), formatCtxPtr_(nullptr), codecCtxPtr_(nullptr),
    videoStreamIndex_(-1), imageConvertCtxPtr_(nullptr), completed_(false)
{
  workerThread_ = thread(&Stream::OpenAndRead, this, streamUrl);
  boost::unique_lock<mutex> lock(mutex_);
  streamOpened_.wait(lock, [this]{ return completed_; });

  if (formatCtxPtr_ == nullptr)
  {
    throw std::runtime_error(error_);
  }
}

int Stream::InterruptCallback(void *ctx)
{
  Stream* streamPtr = reinterpret_cast<Stream*>(ctx);

  if (streamPtr == nullptr)
    return 0;

  {
    boost::lock_guard<boost::mutex> lock(streamPtr->mutex_);
    if (streamPtr->completed_)
        return 0;
  }

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - streamPtr->connectionStart_);

  if (elapsed > streamPtr->connectionTimeout_)
    return 1;

  return 0;
}

void Stream::Open(string const& streamUrl)
{
  static once_flag flag = BOOST_ONCE_INIT;
  call_once(flag, []()
  {
    av_register_all();
    avdevice_register_all();
    avcodec_register_all();
    avformat_network_init();
  });

  formatCtxPtr_ = avformat_alloc_context();
  formatCtxPtr_->interrupt_callback.callback = InterruptCallback;
  formatCtxPtr_->interrupt_callback.opaque = this;
  formatCtxPtr_->flags |= AVFMT_FLAG_NONBLOCK;

  connectionStart_ = std::chrono::system_clock::now();

  int error = avformat_open_input(&formatCtxPtr_, streamUrl.c_str(), nullptr, nullptr);
  if (error != 0)
  {
    throw runtime_error("avformat_open_input() failed: " + AvStrError(error));
  }

  error = avformat_find_stream_info(formatCtxPtr_, nullptr);
  if (error < 0)
  {
    avformat_close_input(&formatCtxPtr_);
    throw runtime_error("avformat_find_stream_info() failed: " + AvStrError(error));
  }

  for (uint32_t i = 0; i < formatCtxPtr_->nb_streams; i++)
  {
    //if (formatCtxPtr_->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
    if (formatCtxPtr_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
    {
      videoStreamIndex_ = i;
      break;
    }
  }

  if (videoStreamIndex_ == -1)
  {
    avformat_close_input(&formatCtxPtr_);
    throw runtime_error("no video stream");
  }

  codecCtxPtr_ = formatCtxPtr_->streams[videoStreamIndex_]->codec;

  AVCodec *codecPtr = avcodec_find_decoder(codecCtxPtr_->codec_id);
  if (codecPtr == nullptr)
  {
    avformat_close_input(&formatCtxPtr_);
    throw runtime_error("avcodec_find_decoder() failed");
  }

  error = avcodec_open2(codecCtxPtr_, codecPtr, nullptr);
  if (error < 0)
  {
    avcodec_close(codecCtxPtr_);
    avformat_close_input(&formatCtxPtr_);
    throw runtime_error("avcodec_open2() failed: " + AvStrError(error));
  }

  // get  SPS, PPS
  //const H264Context *h = (const H264Context *)codecCtxPtr_->priv_data;

}

void Stream::Read()
{
  while (!stopRequested_)
  {
    AVPacket *packetPtr = static_cast<AVPacket *>(av_malloc(sizeof(AVPacket)));
    av_init_packet(packetPtr);
    int error = av_read_frame(formatCtxPtr_, packetPtr);
    if (error < 0)
    {
      //if (error != static_cast<int>(AVERROR_EOF))
      //{
      //    throw runtime_error("av_read_frame() failed: " + AvStrError(error));
      //}

      packetQueue_.Push(nullptr);

      // The end of a stream.
      break;
    }

    if (packetPtr->stream_index == videoStreamIndex_)
    {
      packetQueue_.Push(packetPtr);
    }
    else
    {
      av_packet_unref(packetPtr);
    }
  }
}


void Stream::OpenAndRead(string const& streamUrl)
{
  try
  {
    Open(streamUrl);
  }
  catch (runtime_error &e)
  {
    error_ = e.what();
  }

  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    completed_ = true;
  }

  streamOpened_.notify_one();

  if (formatCtxPtr_ == nullptr || codecCtxPtr_ == nullptr || videoStreamIndex_ == -1)
  {
    return;
  }

  Read();
}

void Stream::PushFrame(BOOL(*fp)(const UINT8&, UINT8 *, UINT32 ))
{
  //while (!sizeq.empty())
  //{
  //}
  //int size = sizeq.front();
  //sizeq.pop(buffer);

  if (stopRequested_ == false)
  {
    int size = singleq.read_available();

    if (size > 0)
    {
      UINT8 * buffer = (UINT8*)malloc(size + 1);
      singleq.pop(buffer);
      fp(0, buffer, size);
      //boost::chrono::milliseconds(100);
      free(buffer);
    }
  }

  boost::chrono::milliseconds(1);
}

void Stream::GetNextFrame()
{
  while (!stopRequested_)
  {
    AVPacket *packetPtr = nullptr;
    if (!packetQueue_.WaitAndPop(packetPtr) || packetPtr == nullptr)
      break;

    singleq.push(packetPtr->buf->data, packetPtr->buf->size);
    boost::chrono::milliseconds(10);
    av_packet_unref(packetPtr);
  }
}

int32_t Stream::InterframeDelayInMilliseconds() const
{
  return codecCtxPtr_->ticks_per_frame * 1000 *
      codecCtxPtr_->time_base.num / codecCtxPtr_->time_base.den;
}

string Stream::AvStrError(int errnum)
{
  char buf[128];
  av_strerror(errnum, buf, sizeof(buf));
  return string(buf);
}

void Stream::Stop()
{
  stopRequested_ = true;
  packetQueue_.StopWait();

  AVPacket *packetPtr = nullptr;
  while (packetQueue_.TryPop(packetPtr))
    av_packet_unref(packetPtr);

  if (workerThread_.joinable())
    workerThread_.join();
}

Stream::~Stream()
{
  if (imageConvertCtxPtr_ != nullptr)
    sws_freeContext(imageConvertCtxPtr_);

  if (codecCtxPtr_ != nullptr)
    avcodec_close(codecCtxPtr_);

  if (formatCtxPtr_ != nullptr)
  {
    avformat_close_input(&formatCtxPtr_);
    avformat_free_context(formatCtxPtr_);
  }
}