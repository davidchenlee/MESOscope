#pragma once
#include <queue>
#include <vector>
//#include <limits.h>		//for _I16_MAX??

typedef   signed    char	I8;
typedef unsigned    char	U8;
typedef            short	I16;
typedef unsigned   short	U16;
typedef              int	I32;
typedef unsigned     int	U32;
typedef          __int64	I64;
typedef unsigned __int64	U63;
typedef std::queue<U32> U32Q;			//Queue of unsigned integers
typedef std::vector<U32Q> U32QV;			//Vector of queues of unsigned integers

											//vibratome channels
typedef enum {
	VTstart,
	VTback,
	VTforward
} VTchannel;


namespace Const
{
	extern const U8 Nchan;
	extern const U8 PCLOCK;
	extern const U8 ABUF0;
	extern const U8 ABUF1;
	extern const U8 ABUF2;
	extern const U8 DBUF0;

	extern const U32 us;
	extern const U32 ms;
	extern const U32 s;
	extern const U32 tickPerUs;
	extern const double dt_us;
	extern const U32 AOdt_us;
	extern const U16 SyncDOtoAO_tick;
	extern const U16 SyncAODOtoLineGate_tick;
	extern const U16 FIFOtimeout;
	extern const U32 FIFOINmax;

	extern const U8 Npulses;
	extern const U8 pulseArray[];

	extern const U16 Width_pixPerFrame;
	extern const U16 Height_pixPerFrame;
	extern const U32 NpixPerFrame;
	extern const U16 NFrames;
	extern const U16 NlinesAllFrames;
	extern const U32 NpixAllFrames;

	extern const U32 FrameWaitTime_tick;
	extern const U32 FrameWaitTime_Latency_tick;

};
