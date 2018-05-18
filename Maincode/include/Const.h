#pragma once
#include <deque>
#include <vector>
#include <array>

//FPGA data types
typedef int8_t		I8;
typedef uint8_t		U8;
typedef int16_t		I16;
typedef uint16_t	U16;
typedef int32_t		I32;
typedef uint32_t	U32;
typedef int64_t		I64;
typedef uint64_t	U64;
typedef std::deque<U32> QU32;				//Queue of unsigned integers
typedef std::vector<QU32> VQU32;			//Vector of queues of unsigned integers
typedef std::array<double, 3> double3;		//array of 3 doubles
typedef std::array<int, 3> int3;			//array of 3 ints

enum PhotonCounterInputSelector {ext, sim};								//real PMT, simulated PMT, internal clock
enum LineclockInputSelector {RS, FG};									//Resonant scanner, function generator, 
enum PixelclockSelector {uniform, corrected};							//uniform dwell times, corrected dwell times
enum RTchannel {PIXELCLOCK, GALVO1, GALVO2, DOdebug, POCKELS1, nChan};	//nChan = number of RT channels available, including the pixel clock channel
enum ShutterID {Shutter1, Shutter2};									//Shutter unit
enum FilterwheelID {FW1, FW2};											//Filterwheel
enum FilterColor { BlueLight = 1, GreenLight = 2, RedLight = 3 };		//Filter color
enum Axis { xx, yy, zz };

namespace Constants
{
	extern const LineclockInputSelector lineclockInput;
	extern const PhotonCounterInputSelector photoncounterInput;
	extern const PixelclockSelector pixelclockType;
	extern const bool overrideImageSaving;

	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int s;
	extern const int um;
	extern const int V;
	extern const int mW;
	extern const int tickPerUs;
	extern const double usPerTick;
	extern const U32 t_tick_MIN;
	extern const double t_us_MIN;
	extern const int AO_t_us_MIN;
	extern const int syncDOtoAO_tick;
	extern const int syncAODOtoLinegate_tick;
	extern const int FIFOtimeout_tick;
	extern const int FIFOINmax;

	extern const int nPulses;
	extern const U8 pulseArray[];
}

namespace Parameters
{
	using namespace Constants;

	extern const std::string foldername;

	extern const double halfPeriodLineclock_us;
	extern const double RSpkpk_um;
	extern const int calibCoarse_tick;
	extern const int calibFine_tick;

	extern const double RS_voltPerUm;

	extern const double upscaleU8;
	extern const int widthPerFrame_pix;
	extern const int heightPerFrame_pix;
	extern const int nPixPerFrame;
	extern const int nLinesSkip;
	extern const int nFrames;
	extern const int nLinesAllFrames;
	extern const int nPixAllFrames;
}