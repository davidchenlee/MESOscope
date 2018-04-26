#pragma once
#include <queue>
#include <vector>
//#include <limits.h>		//for _I16_MAX??

//FPGA data types
typedef int8_t		I8;
typedef uint8_t		U8;
typedef int16_t		I16;
typedef uint16_t	U16;
typedef int32_t		I32;
typedef uint32_t	U32;
typedef int64_t		I64;
typedef uint64_t	U64;
typedef std::queue<U32> QU32;				//Queue of unsigned integers
typedef std::vector<QU32> VQU32;			//Vector of queues of unsigned integers


enum PhotonCounterInputSelector {ext, sim};	//real PMT, simulated PMT, internal clock
enum LineClockInputSelector {RS, FG};					//Resonant scanner, function generator, 
enum RTchannel {PCLOCK, GALVO1, GALVO2, DOdebug};				//Pixel clock, analog channel (galvo 1), analog channel (galvo 2), digital channel (shutter 1)
enum PockelsID {Pockels1, Pockels2};							//PockelsID cell unit
enum FilterwheelID {FW1, FW2};									//Filter wheels

namespace Const
{
	extern const PhotonCounterInputSelector photonCounterInput;
	extern const LineClockInputSelector lineClockInput;

	extern const int Nchan;
	
	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int s;
	extern const int um;
	extern const int V;
	extern const int tickPerUs;
	extern const double dt_us;
	extern const U32 dt_tick_MIN;
	extern const double dt_us_MIN;
	extern const int AOdt_us;
	extern const int syncDOtoAO_tick;
	extern const int syncAODOtoLineGate_tick;
	extern const int FIFOtimeout_tick;
	extern const int FIFOINmax;

	extern const double RS_voltPerUm;
	extern const double galvo_voltPerUm;
	extern const double PC1_voltPermW;
	extern const double PC2_voltPermW;

	extern const int nPulses;
	extern const U8 pulseArray[];

	extern const double halfPeriodLineClock_us;
	extern const double RSpkpk_um;

	extern const double fFOVslow_um;
	extern const double galvo1Amp_volt;

	extern const double galvoTimeStep_us;
	extern const int widthPerFrame_pix;
	extern const int heightPerFrame_pix;
	extern const int nPixPerFrame;
	extern const int nLinesSkip;
	extern const int nFrames;
	extern const int nLinesAllFrames;
	extern const int nPixAllFrames;

}