#include "streamplayer.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <stdexcept>

#define STREAMPLAYER_API extern "C"

FFmpeg::Facade::StreamPlayer player;

STREAMPLAYER_API int32_t __stdcall Initialize(FFmpeg::Facade::StreamPlayerParams params, BOOL(*fp)(const UINT8&, UINT8 *, UINT32 ))
{
  try
  {
    player.Initialize(params, fp);
  }
  catch (std::runtime_error &)
  {
    return 1;
  }

  return 0;
}

#include <string>
STREAMPLAYER_API int32_t __stdcall StartPlay(std::string const &url, uint32_t connectionTimeoutInMilliseconds)
{
  try
  {
    player.StartPlay(url, connectionTimeoutInMilliseconds);
  }
  catch (std::runtime_error &)
  {
    return 1;
  }

  return 0;
}

STREAMPLAYER_API int32_t __stdcall Stop()
{
  try
  {
    player.Stop();
  }
  catch (std::runtime_error &)
  {
    return 1;
  }

  return 0;
}

STREAMPLAYER_API int32_t __stdcall Uninitialize()
{
  try
  {
    player.Uninitialize();
  }
  catch (std::runtime_error &)
  {
    return 1;
  }

  return 0;
}