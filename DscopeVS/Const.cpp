#include "Const.h"

namespace Const
{
	extern const unsigned char Nchan = 3;
	extern const unsigned char AO0 = 0;
	extern const unsigned char AO1 = 1;
	extern const unsigned char DO0 = 2;

	extern const unsigned int tick = 1;
	extern const unsigned int us = 1;
	extern const unsigned int ms = 1000 * us;
	extern const unsigned int s = 1000000 * us;
	//extern const unsigned int V = 1;
	extern const unsigned int tickPerUs = 40;	
	extern const unsigned int AO_dt = 4 * us;
	extern const U16 DODelayTick = 80 * tick;



};