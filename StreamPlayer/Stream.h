#ifndef FFMPEG_FACADE_STREAM_H
#define FFMPEG_FACADE_STREAM_H

#include <cstdint>
#include <string>
#include <memory>
#include <chrono>
#include <boost/noncopyable.hpp>

#pragma warning( push )
#pragma warning( disable : 4100 )

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <boost/thread.hpp>

#pragma warning( pop )

#include <queue>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/atomic.hpp>

#include "ConcurrentQueue.h"


namespace FFmpeg
{

#pragma warning( push )
#pragma warning( disable : 4244 )

	extern "C"
	{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
	}

// Get H264 SPS, PPS
#include <h264_define.h>

#pragma warning( pop )

	// The FFmpeg framework built using the following configuration:
  // x64: ./configure --toolchain=msvc --arch=amd64 --target-os=win64 --enable-version3  --enable-x86asm --enable-asm --disable-static --enable-shared --disable-programs --disable-doc  
  // x86:  ./configure --toolchain=msvc --arch=i386 --enable-version3  --enable-x86asm --enable-asm --disable-static --enable-shared --disable-programs --disable-doc 

#pragma comment(lib, "avformat.lib")
#pragma comment(lib, "avcodec.lib")
#pragma comment(lib, "avdevice.lib")
#pragma comment(lib, "avfilter.lib")
#pragma comment(lib, "avutil.lib")
#pragma comment(lib, "swresample.lib")
#pragma comment(lib, "swscale.lib")
#pragma comment(lib, "WS2_32.lib")
#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "Vfw32.lib")
#pragma comment(lib, "Shlwapi.lib")
#pragma comment(lib, "Secur32.lib")

//#define RTSP_BUFFER 0x2000000 //33.5MB
#define RTSP_BUFFER 0x4000000 //67MB

	namespace Facade
	{
		class Frame;

		/// <summary>
		/// A Stream class converts a stream into a set of frames.
		/// </summary>
		class Stream : private boost::noncopyable
		{
		public:
			/// <summary>
			/// Initializes a new instance of the Stream class.
			/// </summary>
			/// <param name="streamUrl">The url of a stream to decode.</param>
            /// <param name="connectionTimeoutInMilliseconds">The connection timeout in milliseconds.</param>
            Stream(std::string const& streamUrl,
                int32_t connectionTimeoutInMilliseconds, BOOL(*fp)(const UINT8&, UINT8 *, UINT32 ));

			/// <summary>
			/// Gets the next frame in the stream.
			/// </summary>
			/// <returns>The next frame in the stream or nullptr if there are no more frames.</returns>

            void GetNextFrame(void);
            void PushFrame();


			/// <summary>
			/// Gets an interframe delay, in milliseconds.
			/// </summary>
			int32_t InterframeDelayInMilliseconds() const;

            void Stop();

			/// <summary>
			/// Releases all resources used by the decoder.
			/// </summary>
			~Stream();

		private:

      BOOL(*stream_fp)(const UINT8&, UINT8 *, UINT32);

      void Open(std::string const& streamUrl);

      void Read();

      void OpenAndRead(std::string const& streamUrl);

      static int InterruptCallback(void *ctx);

      static std::string AvStrError(int errnum);

      std::chrono::milliseconds connectionTimeout_;
      boost::atomic<bool> stopRequested_;

      AVFormatContext *formatCtxPtr_;
      AVCodecContext  *codecCtxPtr_;
      AVCodecParameters *codecparPtr_;

      int32_t videoStreamIndex_;
      SwsContext *imageConvertCtxPtr_;

      bool completed_;
      std::string error_;
      boost::thread workerThread_;
      boost::mutex mutex_;
      boost::condition_variable streamOpened_;

#if 0
      boost::mutex m_;
      boost::condition_variable cv_;
      std::queue <int> sizeq;
#endif

      boost::lockfree::spsc_queue<UINT8, boost::lockfree::capacity<RTSP_BUFFER> > singleq;

      ConcurrentQueue<AVPacket *> packetQueue_;

      std::chrono::time_point<std::chrono::system_clock> connectionStart_;
		};
	}
}

#endif // FFMPEG_FACADE_STREAM_H