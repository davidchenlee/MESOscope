#pragma once
#include <queue>
#include <vector>
//#include <limits.h>		//for _I16_MAX??

//FPGA data types
typedef unsigned char NiFpga_Bool;
typedef   signed    char	I8;
typedef unsigned    char	U8;
typedef            short	I16;
typedef unsigned   short	U16;
typedef              int	I32;
typedef unsigned     int	U32;
typedef          __int64	I64;
typedef unsigned __int64	U63;
typedef std::queue<U32> QU32;				//Queue of unsigned integers
typedef std::vector<QU32> VQU32;			//Vector of queues of unsigned integers


enum PhotonCounterInputSelector { PMText, PMTsim, FPGAclock };	//real PMT, simulated PMT, internal clock
enum LineClockInputSelector { ResScan, FuncGen };	//Resonant scanner, function generator, 
enum RTchannel {PCLOCK, GALVO1, GALVO2, SHUTTER1};		//Pixel clock, analog channel (galvo 1), analog channel (galvo 2), digital channel (shutter 1)

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
