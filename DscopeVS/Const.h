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
typedef std::queue<uint32_t> U32Q;			//Queue of unsigned integers
typedef std::vector<U32Q> U32QV;			//Vector of queues of unsigned integers

namespace Const
{
	extern const U8 Nchan;
	extern const U8 AO0;
	extern const U8 AO1;
	extern const U8 DO0;
	extern const U8 DO1;
	extern const U8 PCLOCK;
	extern const U8 Ncounters;

	extern const U32 us;
	extern const U32 ms;
	extern const U32 s;
	extern const U32 tickPerUs;
	extern const double dt_us;
	extern const U32 AOdt_us;
	extern const U16 Sync_DO_to_AO;
	extern const U16 Sync_AODO_to_LineGate;
	extern const U16 FIFOtimeout;

	extern const U8 Npulses;
	extern const U8 pulseArray[];

	extern const I16 Nmaxlines;
	extern const U16 Npixels;
};
