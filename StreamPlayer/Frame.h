#ifndef FFMPEG_FACADE_FRAME_H
#define FFMPEG_FACADE_FRAME_H

#include <cstdint>
#include <stdexcept>
#include <boost/noncopyable.hpp>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

//#define WIN32API  1 //WIN32API
//#define D2D             1 //Direct2D
#define D3D             1 //Direct3D
//#define DSH            1 //DirectShow //not worked

#ifdef D2D
#include "../device/include/DIRECT2D.h"
#elif D3D
#include "../device/include/DIRECT3D.h"
#elif DSH
#include "DSHOW.h"
#endif


namespace FFmpeg
{

#pragma warning( push )
#pragma warning( disable : 4244 )

    extern "C"
    {
#include <libavcodec/avcodec.h>
    }

#pragma warning( pop )

    namespace Facade
    {
        /// <summary>
        /// A Frame class implements a set of frame-related utilities.
        /// </summary>
        class Frame : private boost::noncopyable
        {
        public:
            /// <summary>
            /// Initializes a new instance of the Frame class.
            /// </summary>
            /// <param name="width">The width of the frame, in pixels.</param>
            /// <param name="height">The height of the frame, in pixels.</param>
            /// <param name="avFrame">The AVFrame object to initialize the frame with.</param>
            Frame(uint32_t width, uint32_t height, AVFrame &avFrame);

            /// <summary>
            /// Gets the width of the frame, in pixels.
            /// </summary>
            uint32_t Width() const { return width_; }

            /// <summary>
            /// Gets the height of the frame, in pixels.
            /// </summary>
            uint32_t Height() const { return height_; }

            /// <summary>
            /// Draws the frame.
            /// </summary>
            /// <param name="window">A container window that frame should be drawn on.</param>
            void Draw(HWND window);

            /// <summary>
            /// Converts the frame to a bitmap.
            /// </summary>
            /// <param name="bmpPtr">Address of a pointer to a byte that will receive the DIB.</param>
            void ToBmp(uint8_t **bmpPtr);

            /// <summary>
            /// Releases all resources used by the frame.
            /// </summary>
            ~Frame()
            {
              delete[] pixelsPtr_;
            }

        private:
            /// <summary>
#if 0
            /// The bits in the array are packed together, but each scan line must be
            /// padded with zeros to end on a LONG data-type boundary.
            /// </summary>
            uint32_t GetPadding(int32_t lineSize) const
            {
#ifdef WIN32API
              return lineSize % sizeof(uint32_t) > 0 ?
                sizeof(uint32_t) - (lineSize % sizeof(uint32_t)) : 0;
#elif DSH
              //return lineSize % sizeof(uint32_t) > 0 ?
                //sizeof(uint32_t) - (lineSize % sizeof(uint32_t)) : 0;
              return (lineSize > 1024) ?
                (2048 - lineSize) : (1024 - lineSize);
#else
              //return (lineSize > 1024) ?
                //(2048 - lineSize) : (1024 - lineSize);

              int mod = lineSize % 32;

              switch (mod)
              {
              case 0:
                return 0;
              default:
                return (32 - mod);
              }
#endif
            }
#endif

            int32_t width_, height_;
            uint8_t *pixelsPtr_;
            int32_t  pitch_;

#ifdef D2D
            DIRECT2D *m_render;
#elif D3D
            DIRECT3D *m_render;
#endif

        };
    }
}

#endif // FFMPEG_FACADE_FRAME_H