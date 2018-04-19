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
typedef std::queue<U32> U32Q;				//Queue of unsigned integers
typedef std::vector<U32Q> U32QV;			//Vector of queues of unsigned integers

//vibratome channels
enum VibratomeChannel {
	VibratomeStart,
	VibratomeBack,
	VibratomeForward
} ;

enum PhotonCounterInputSelector {
	PMText = 0,								//0 for the real external PMT
	PMTsim = 1								//1 for the simulated PMT
};

enum LineClockInputSelector {
	ResScan = 0,							//0 for the resonant scanner
	FuncGen = 1,							//1 for the function generator
	FPGAclock = 2
};


namespace Const
{
	extern const int Nchan;
	extern const int PCLOCK;
	extern const int ABUF0;
	extern const int ABUF1;
	extern const int ABUF2;
	extern const int DBUF0;

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

	extern double *PixelClockEqualDistanceLUT;
	extern const double HalfPeriodLineClock_us;
	extern const double RSamplitudePkPK_um;
};

