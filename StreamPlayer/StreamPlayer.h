#pragma once

#include <string>
#include <memory>
#include <boost/noncopyable.hpp>
#pragma warning( push )
#pragma warning( disable : 4100 )
#include <boost/thread.hpp>
#pragma warning( pop )
#include <boost/atomic.hpp>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include "stream.h"

namespace FFmpeg
{
  namespace Facade
  {
    class Stream;

    struct StreamPlayerParams
    {
    StreamPlayerParams()
	    : window(nullptr)
      {}
      HWND window;
    };

    /// <summary>
    /// A StreamPlayer class implements a stream playback functionality.
    /// </summary>
    class StreamPlayer : private boost::noncopyable
    {
    public:

        /// <summary>
        /// Initializes a new instance of the StreamPlayer class.
        /// </summary>
        StreamPlayer();

        /// <summary>
        /// Initializes the player.
        /// </summary>
			  /// <param name="playerParams">The StreamPlayerParams object that contains the information that is used to initialize the player.</param>
        void Initialize(StreamPlayerParams playerParams, BOOL(*fp)(const UINT8& ch_id, UINT8 *buf, UINT32 buf_size));

        /// <summary>
        /// Asynchronously plays a stream.
        /// </summary>
        /// <param name="streamUrl">The url of a stream to play.</param>
        /// <param name="connectionTimeoutInMilliseconds">The connection timeout in milliseconds.</param>
        void StartPlay(std::string const& streamUrl,
            uint32_t connectionTimeoutInMilliseconds);

        /// <summary>
        /// Stops a stream.
        /// </summary>
        void Stop();

        /// <summary>
        /// Uninitializes the player.
        /// </summary>
        void Uninitialize();

    private:
			  /// <summary>
			  /// Plays a stream.
			  /// </summary>
			  /// <param name="streamUrl">The url of a stream to play.</param>
        /// <param name="connectionTimeoutInMilliseconds">The connection timeout in milliseconds.</param>
        void Produce(std::string const& streamUrl,  int32_t connectionTimeoutInMilliseconds);
        void Consume(void);

        static LRESULT APIENTRY WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    private:
        BOOL  (*fp)(const UINT8& ch_id, UINT8 *buf, UINT32 buf_size);

        boost::atomic<bool> stopRequested_;
        StreamPlayerParams playerParams_;

        boost::mutex streamMutex_;
        std::unique_ptr<Stream> streamPtr_;

        // There is a bug in the Visual Studio std::thread implementation,
        // which prohibits dll unloading, that is why the boost::thread is used instead.
        // https://connect.microsoft.com/VisualStudio/feedback/details/781665/stl-using-std-threading-objects-adds-extra-load-count-for-hosted-dll#tabs

        boost::mutex pThreadMutex_;
			  boost::thread pThread_;

        boost::mutex cThreadMutex_;
			  boost::thread cThread_;

        boost::mutex errorMutex_;
        std::string error_;

        static WNDPROC originalWndProc_;
    };
  }
}
