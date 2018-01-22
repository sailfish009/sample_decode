#include "streamplayer.h"
#include <cassert>

#define WM_INVALIDATE    WM_USER + 1
#define WM_STREAMSTARTED WM_USER + 2
#define WM_STREAMSTOPPED WM_USER + 3
#define WM_STREAMFAILED  WM_USER + 4

using namespace std;
using namespace boost;
using namespace FFmpeg;
using namespace FFmpeg::Facade;

WNDPROC StreamPlayer::originalWndProc_ = nullptr;

StreamPlayer::StreamPlayer()
	: stopRequested_(false) {}

void StreamPlayer::Initialize(StreamPlayerParams params, BOOL(*fpp)(const UINT8&, UINT8 *, UINT32 ))
{
  assert(params.window != nullptr);
  playerParams_ = params;
  if (playerParams_.window == nullptr)
    return;

  ::SetWindowLongPtr(playerParams_.window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
  originalWndProc_ = reinterpret_cast<WNDPROC>(::SetWindowLongPtr(playerParams_.window, GWLP_WNDPROC,
    reinterpret_cast<LONG_PTR>(WndProc)));

  fp = fpp;
}

void StreamPlayer::StartPlay(string const& streamUrl,
    uint32_t connectionTimeoutInMilliseconds)
{
	pThread_ = boost::thread(&StreamPlayer::Produce, this,
        streamUrl, connectionTimeoutInMilliseconds);

  cThread_ = boost::thread(&StreamPlayer::Consume, this);
}

void StreamPlayer::Consume()
{
  boost::unique_lock<boost::mutex> lock(cThreadMutex_, boost::defer_lock);
  if (!lock.try_lock())
  {
    // Skip subsequent calls until the stream fails or stopped.
    return;
  }

  try
  {
    {
      unique_lock<mutex> lock(streamMutex_);
    }

    for (;;)
    {

      streamPtr_->PushFrame();

      if (stopRequested_)
      {
        if (playerParams_.window != nullptr)
        {
          ::PostMessage(playerParams_.window, WM_STREAMSTOPPED, 0, 0);
        }
        break;
      }

      if (playerParams_.window != nullptr)
        ::PostMessage(playerParams_.window, WM_INVALIDATE, 0, 0);

      boost::this_thread::sleep_for
      (
        boost::chrono::milliseconds(streamPtr_->InterframeDelayInMilliseconds())
      );
    }

    {
      unique_lock<mutex> lock(streamMutex_);
      streamPtr_.reset();
    }
  }
  catch (runtime_error& e)
  {
    {
      unique_lock<mutex> lock(errorMutex_);
      error_ = e.what();
    }

    if (playerParams_.window != nullptr)
    {
      ::PostMessage(playerParams_.window, WM_STREAMFAILED, 0, 0);
    }
  }



}

void StreamPlayer::Produce(string const& streamUrl,
    int32_t connectionTimeoutInMilliseconds)
{
  boost::unique_lock<boost::mutex> lock(pThreadMutex_, boost::defer_lock);

  if (!lock.try_lock())
  {
    // Skip subsequent calls until the stream fails or stopped.
    return;
  }

  try
  {
    {
      unique_lock<mutex> lock(streamMutex_);
      streamPtr_ = make_unique<Stream>(streamUrl, connectionTimeoutInMilliseconds, fp);
    }

    stopRequested_ = false;

    for (;;)
    {
      streamPtr_->GetNextFrame();

	    if (stopRequested_ )
	    {
        if (playerParams_.window != nullptr)
        {
          ::PostMessage(playerParams_.window, WM_STREAMSTOPPED, 0, 0);
        }
		    break;
	    }

      if (playerParams_.window != nullptr)
        ::PostMessage(playerParams_.window, WM_INVALIDATE, 0, 0);

      boost::this_thread::sleep_for
      (
          boost::chrono::milliseconds(streamPtr_->InterframeDelayInMilliseconds())
      );
    }

    {
      unique_lock<mutex> lock(streamMutex_);
      streamPtr_.reset();
    }
  }
  catch (runtime_error& e)
  {
    {
      unique_lock<mutex> lock(errorMutex_);
      error_ = e.what();
    }

    if (playerParams_.window != nullptr)
    {
      ::PostMessage(playerParams_.window, WM_STREAMFAILED, 0, 0);
    }
  }

}

void StreamPlayer::Stop()
{
  stopRequested_ = true;

  {
    unique_lock<mutex> lock(streamMutex_);
    if (streamPtr_ != nullptr)
    {
      streamPtr_->Stop();
    }
  }

  if (pThread_.joinable())
      pThread_.join();

  if (cThread_.joinable())
      cThread_.join();

}

void StreamPlayer::Uninitialize()
{
  Stop();

  if (playerParams_.window != nullptr && originalWndProc_ != nullptr)
  {
    // Clear the message queue.
    MSG msg;
    while (::PeekMessage(&msg, playerParams_.window, 0, 0, PM_REMOVE)) {}

    ::SetWindowLongPtr(playerParams_.window, GWLP_USERDATA, 0);
    ::SetWindowLongPtr(playerParams_.window, GWLP_WNDPROC,
        reinterpret_cast<LONG_PTR>(originalWndProc_));
  }
}

LRESULT APIENTRY StreamPlayer::WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  static StreamPlayer *playerPtr = nullptr;
  if (playerPtr == nullptr)
  {
    playerPtr = reinterpret_cast<StreamPlayer *>(GetWindowLongPtr(hWnd, GWLP_USERDATA));
    assert(playerPtr != nullptr);
  }

  switch (uMsg)
  {
  case WM_INVALIDATE:
    ::InvalidateRect(hWnd, nullptr, FALSE);
    break;

  default:
    break;
  }

  return CallWindowProc(originalWndProc_, hWnd, uMsg, wParam, lParam);
}


