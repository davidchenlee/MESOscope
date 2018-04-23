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


enum PhotonCounterInputSelector {
	PMText = 0,								//0 for the real external PMT
	PMTsim = 1								//1 for the simulated PMT
};

enum LineClockInputSelector {
	ResScan = 0,							//0 for the resonant scanner
	FuncGen = 1,							//1 for the function generator
	FPGAclock = 2
};

enum RTchannel {
	PCLOCK = 0,								//Pixel clock
	IDgalvo1 = 1,							//Analog channel (galvo 1)
	IDgalvo2 = 2,							//Analog channel (galvo 2)
	IDshutter1 = 3							//Digital channel(shutter 1)
};


namespace Const
{
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
	extern const int SyncDOtoAO_tick;
	extern const int SyncAODOtoLineGate_tick;
	extern const int FIFOtimeout_tick;
	extern const int FIFOINmax;

	extern const int WidthPerFrame_pix;
	extern const int HeightPerFrame_pix;
	extern const int NpixPerFrame;
	extern const int NlinesSkip;
	extern const int NFrames;
	extern const int NlinesAllFrames;
	extern const int NpixAllFrames;

	extern const PhotonCounterInputSelector PhotonCounterInput;
	extern const LineClockInputSelector LineClockInput;

	extern const double RS_voltPerUm;
	extern const double Galvo_voltPerUm;
	extern const double PC1_voltPermW;
	extern const double PC2_voltPermW;

	extern const int Npulses;
	extern const U8 pulseArray[];

	extern std::vector<double> PixelClockEqualDistanceLUT;
	extern const double HalfPeriodLineClock_us;
	extern const double RSamplitudePkPK_um;
};

