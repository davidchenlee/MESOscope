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
	extern const unsigned char Nchan;		//Number of channels available. WARNING: This number has to match the implementation on the FPGA!
	extern const unsigned char AO0;
	extern const unsigned char AO1;
	extern const unsigned char DO0;


	extern const unsigned int tick;			//one tick of the FPGA clock
	extern const unsigned int us;			//microsecond
	extern const unsigned int ms;			//millisecond
	extern const unsigned int s;			//second
	//extern const unsigned int V;
	extern const unsigned int tickPerUs;	//Number of ticks in 1 us. Depends on the FPGA hardware clock
	extern const unsigned int AO_dt;		//Analog output time increament in us. Currently, the AO works well with 4us or more
	extern const U16 DODelayTick;			//relative delay between AO and DO. This is because the AO takes longer than DO to write the output


};
