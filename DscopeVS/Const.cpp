#include "Const.h"

namespace Const
{
	extern const unsigned char Nchan = 3;		//Number of channels available. WARNING: This number has to match the implementation on the FPGA!
	extern const unsigned int tick = 1;
	extern const unsigned int us = 1;
	extern const unsigned int ms = 1000 * us;
	extern const unsigned int s = 1000000 * us;
	//extern const unsigned int V = 1;

	extern const unsigned int tickPerUs = 40;	//Number of ticks in 1 us. Depends on the FPGA hardware clock
	extern const unsigned int AO_dt = 4 * us;	//Analog output time increament in us. Currently, the AO works well with 4us or more

	extern const U16 DODelayTick = 80 * tick; //relative delay between AO and DO. This is because the AO takes longer than DO to write the output


};