#pragma once
#include <deque>
#include <vector>
#include <array>

namespace Constants
{
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
	typedef std::array<double, 3> double3;		//array of 3 doubles. Arrays allow to allocate memory. Vectors don't
	typedef std::array<int, 3> int3;			//array of 3 ints

	enum PhotonCounterSelector { EXT, SIM };
	enum LineclockSelector { RS = false, FG = true };
	enum ToggleSwitch { DISABLE = false, ENABLE = true};
	enum TiffPageStruct { SINGLEPAGE = false, MULTIPAGE = true};
	enum OverrideFile { NOOVERRIDE = false, OVERRIDE = true};
	enum PixelclockSelector { UNIFORM, NONUNIFORM };
	enum RTchannel { PIXELCLOCK, GALVO1, GALVO2, DOdebug, POCKELS1, SCALING1, nChan };		//nChan = number of RT channels available, including the pixel clock channel
	enum ShutterID { SHUTTER1 = 1, SHUTTER2 };
	enum FilterwheelID { FW1 = 1, FW2 };
	enum Filtercolor { BLUE = 1, GREEN, RED };
	enum Axis { xx, yy, zz };
	enum RunMode { singleRM, contRM, averageRM, stackRM, stackCenterRM };

	enum ComID { VISIONcom, FW1com , FW2com, mPMTcom, ZSTAGEcom };
	extern const std::vector<std::string> assignCOM;

	extern const std::string folderPath;
	extern const std::string bitfilePath;

	extern const PhotonCounterSelector photoncounterInput;
	extern const PixelclockSelector pixelclockType;
	extern const ToggleSwitch FIFOOUTfpgaEnable;
	extern const ToggleSwitch pockels1AutoOffEnable;

	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int s;
	extern const int um;
	extern const int V;
	extern const int mW;

	extern const double halfPeriodLineclock_us;
	extern const double VISIONpulsePeriod;

	extern const int AOmax_V;
	extern const int tickPerUs;
	extern const double usPerTick;
	extern const U32 tMIN_tick;
	extern const double tMIN_us;
	extern const int AO_tMIN_us;
	extern const int syncDOtoAO_tick;
	extern const int syncAODOtoLinegate_tick;
	extern const double linegateTimeout_us;
	extern const int FIFOINtimeout_tick;
	extern const int FIFOINmax;

	extern const int stageTriggerPulse_ms;

	extern const int nPulses;
	extern const U8 pulseArray[];
}