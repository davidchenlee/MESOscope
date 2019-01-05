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
	typedef std::array<double, 3> double3;		//array of 3 doubles. Arrays allow pre-defining their size. Vectors do not
	typedef std::array<int, 3> int3;			//array of 3 ints
	typedef std::array<int, 2> int2;			//array of 3 ints
	typedef std::array<double, 2> double2;		//array of 2 doubles
	typedef std::array <double, 4> ROI;			//ROI = (ymin, xmin, ymax, xmax)

	enum PhotonCounterSelector { EXT, SIM };
	enum LineclockSelector { RS = false, FG = true };
	enum AcqTriggerSelector { PCTRIG = false, STAGETRIG = true};
	enum ToggleSwitch { DISABLE = false, ENABLE = true};
	enum FPGAresetSelector { NORESET = false, RESET = true };
	enum TiffPageStructSelector { SINGLEPAGE = false, MULTIPAGE = true};
	enum OverrideFileSelector { NOOVERRIDE = false, OVERRIDE = true};
	enum PixelclockSelector { UNIFORM, NONUNIFORM };
	enum RTchannel { RTPIXELCLOCK, RTGALVO1, RTGALVO2, RTDODEBUG, RTVISION, RTSCALINGVISION, RTFIDELITY, RTSCALINGFIDELITY, RTNCHAN };		//RTNCHAN = number of RT channels available, including the channel for the pixelclock
	enum FilterwheelID { FWDET = 1, FWEXC = 2 };
	enum Axis { XX, YY, ZZ };
	enum RunMode { SINGLEMODE, LIVEMODE, AVGMODE, STACKMODE, STACKCENTEREDMODE };
	enum Action { CUT, ACQ, SAV, MOV };
	enum StackScanDir { BOTTOMUP = -1, TOPDOWN = 1 };
	enum LaserSelector { VISION, FIDELITY, AUTO};
	enum Filtercolor { BLUE, GREEN, RED, NONE };

	enum ComID { COMVISION, COMFIDELITY, COMFWDET , COMFWEXC, COMPMT16X };
	extern const std::vector<std::string> assignCOM;

	extern const std::string folderPath;
	extern const std::string bitfilePath;

	extern const PhotonCounterSelector photoncounterInput;
	extern const PixelclockSelector pixelclockType;
	extern const ToggleSwitch FIFOOUTfpga;
	extern const ToggleSwitch pockelsAutoOff;

	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int sec;
	extern const int um;
	extern const int mm;
	extern const double mmps;
	extern const int V;
	extern const int mW;

	extern const double halfPeriodLineclock;
	extern const double VISIONpulsePeriod;

	extern const int AOmax;
	extern const int tickPerUs;
	extern const double usPerTick;
	extern const U32 tMIN_tick;
	extern const int AO_tMIN;
	extern const int syncDOtoAO_tick;
	extern const int syncAODOtoLinegate_tick;
	extern const double linegateTimeout;
	extern const int FIFOINtimeout_tick;
	extern const int FIFOINmax;

	extern const int stageTriggerPulse;

	extern const int nPulses;
	extern const U8 pulseArray[];
}