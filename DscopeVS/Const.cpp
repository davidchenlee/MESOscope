#include "Const.h"

namespace Const
{
	extern const unsigned char Nchan = 4;				//Number of channels available. WARNING: This number has to match the implementation on the FPGA!
	extern const unsigned char AO0 = 0;
	extern const unsigned char AO1 = 1;
	extern const unsigned char DO0 = 2;

	extern const unsigned int us = 1;					//microsecond
	extern const unsigned int ms = 1000 * us;			//millisecond
	extern const unsigned int s = 1000000 * us;			//second
	//extern const unsigned int V = 1;
	extern const unsigned int tickPerUs = 160;			//Number of ticks in 1 us. It depends on the FPGA's clock
	extern const unsigned int AO_dt = 2 * us;			//Time step (in us) of the analog output. The AO channels take >1us to write the output
	extern const U16 DODelayTick = 4*74;				//relative delay between AO and DO. This is because AO takes longer than DO to write the output
	extern const unsigned int FIFOtimeout = 100;		//in ticks


};