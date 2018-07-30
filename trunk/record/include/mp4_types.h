/**
 * mp4_types.h
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

#ifndef __MP4_TYPES_H__
#define __MP4_TYPES_H__


#include <stdint.h>
#include <stdbool.h>

typedef unsigned int UINT;
typedef int          INT;
typedef volatile int am_atomic_t;

typedef uint8_t      U8;
typedef uint16_t     U16;
typedef uint32_t     U32;
typedef uint64_t     U64;

typedef int8_t       S8;
typedef int16_t      S16;
typedef int32_t      S32;
typedef int64_t      S64;

typedef U64       	 PTS;
typedef S32          file_off_t;
typedef uint8_t      char_t;

typedef bool         BOOL;
//++#define TRUE         true
//++#define FALSE        false

//-----------------------------------------------------------------------
//
//	error code
//
//-----------------------------------------------------------------------

typedef enum ERR
{
  ME_OK = 0,

  ME_PENDING,
  ME_ERROR,
  ME_CLOSED,
  ME_BUSY,
  ME_NO_IMPL,
  ME_OS_ERROR,
  ME_IO_ERROR,
  ME_FILE_END,
  ME_TIMEOUT,
  ME_NO_MEMORY,
  ME_TOO_MANY,

  ME_NOT_EXIST,
  ME_NOT_SUPPORTED,
  ME_NO_INTERFACE,
  ME_BAD_STATE,
  ME_BAD_PARAM,
  ME_BAD_COMMAND,

  ME_BAD_FORMAT,
  ME_NO_ACTION,
}ERR;

typedef enum MM_MEDIA_FORMAT
{
  MF_NULL,
  MF_TEST,

  MF_H264,
  MF_MJPEG,

  MF_PCM,
  MF_G711,
  MF_G726_40,
  MF_G726_32,
  MF_G726_24,
  MF_G726_16,
  MF_AAC,
  MF_OPUS,
  MF_BPCM,

  MF_TS,
  MF_MP4,
}MM_MEDIA_FORMAT;

typedef struct  {
    U32 needsync; /* Indicate if A/V sync is needed in AVQueue */
    U32 type;  /* Video encode type: 1: H.264, 2: MJPEG */
    U32 fps;   /* framerate = 512000000 / fps */
    U32 rate;  /* Video rate */
    U32 scale; /* Video scale, framerate = scale/rate, 29.97 = 30000/1001 */
    U16 mul;   /* Video framerate numerator */
    U16 div;   /* Video framerate demoninator */
    U16 devidor;
    U16 width;
    U16 height;
    U16 M;
    U16 N;
}AM_VIDEO_INFO;

typedef struct  {
    UINT needsync;       /* Indicate if A/V sync is needed in AVQueue */
    UINT sampleRate;
    UINT channels;
    UINT pktPtsIncr;
    UINT sampleSize;
    UINT chunkSize;
    MM_MEDIA_FORMAT format;
}AM_AUDIO_INFO;

//-----------------------------------------------------------------------
//
//	time, delay
//
//-----------------------------------------------------------------------
#include <time.h>
//#include <sys/time.h>

#define AM_MSLEEP(_msec) \
    do { \
      struct timespec req; \
      time_t sec = _msec /1000; \
      req.tv_sec = sec; \
      req.tv_nsec = (_msec - sec * 1000) * 1000000L; \
      nanosleep(&req, NULL); \
    } while (0)

#ifndef timersub
#define	timersub(a, b, result) \
		do { \
			(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
			(result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
			if ((result)->tv_usec < 0) { \
				--(result)->tv_sec; \
				(result)->tv_usec += 1000000; \
			} \
		} while (0)
#endif

#ifndef timermsub
#define	timermsub(a, b, result) \
    do { \
      (result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
      (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
      if ((result)->tv_nsec < 0) { \
        --(result)->tv_sec; \
        (result)->tv_nsec += 1000000000L; \
      } \
    } while (0)
#endif

//-----------------------------------------------------------------------
//
//	macros
//
//-----------------------------------------------------------------------

// align must be power of 2
#ifndef ROUND_UP
#define ROUND_UP round_up
#endif

#ifndef ROUND_DOWN
#define ROUND_DOWN round_down
#endif

#ifndef AM_MIN
#define AM_MIN(_a, _b)			((_a) < (_b) ? (_a) : (_b))
#endif

#ifndef AM_MAX
#define AM_MAX(_a, _b)			((_a) > (_b) ? (_a) : (_b))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_array)		(sizeof(_array) / sizeof(_array[0]))
#endif

#ifndef OFFSET
#define OFFSET(_type, _member)		((int)&((_type*)0)->member)
#endif

#ifndef PTR_ADD
#define PTR_ADD(_ptr, _size)		(void*)((char*)(_ptr) + (_size))
#endif

#ifndef ABS
#define ABS(x) (((x) < 0) ? -(x) : (x))
#endif


#endif

