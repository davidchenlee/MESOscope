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
	extern const unsigned char Nchan;
	extern const unsigned char AO0;
	extern const unsigned char AO1;
	extern const unsigned char DO0;

	extern const unsigned int us;
	extern const unsigned int ms;
	extern const unsigned int s;			
	//extern const unsigned int V;
	extern const unsigned int tickPerUs;
	extern const unsigned int AO_dt;
	extern const U16 DODelayTick;
	extern const unsigned int FIFOtimeout;


};
