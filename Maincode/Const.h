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
typedef enum {
	VibratomeStart,
	VibratomeBack,
	VibratomeForward
} VibratomeChannel;


namespace Const
{
	extern const int Nchan;
	extern const int PCLOCK;
	extern const int ABUF0;
	extern const int ABUF1;
	extern const int ABUF2;
	extern const int DBUF0;

	extern const int us;
	extern const int ms;
	extern const int s;
	extern const int um;
	extern const int V;
	extern const int tickPerUs;
	extern const double dt_us;
	extern const int AOdt_us;
	extern const int SyncDOtoAO_tick;
	extern const int SyncAODOtoLineGate_tick;
	extern const int FIFOtimeout;
	extern const int FIFOINmax;

	extern const int WidthPerFrame_pix;
	extern const int HeightPerFrame_pix;
	extern const int NpixPerFrame;
	extern const int NFrames;
	extern const int NlinesAllFrames;
	extern const int NpixAllFrames;

	extern const double RS_voltPerUm;
	extern const double Galvo_voltPerUm;
	extern const double PC1_voltPermW;
	extern const double PC2_voltPermW;



	extern const int Npulses;
	extern const U8 pulseArray[];
};
