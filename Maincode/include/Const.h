#pragma once
#include <deque>
#include <vector>
#include <array>

#define multibeam 1			//Multibeam or singlebeam
#define pockelsAutoOff 1	//For debugging purposes. In LV, let 'framegate' gate the output of the pockels cell

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
	typedef std::array <double, 4> ROI;			//ROI = {ymin, xmin, ymax, xmax}

	enum class INPUT { PMT, SIM };
	enum class LINECLOCK { RS, FG  };
	enum class MAINTRIG { PC, ZSTAGE };
	enum class FPGARESET { DIS, EN };
	enum class FIFOOUT { DIS, EN  };
	enum class MULTIPAGE { DIS, EN };
	enum class OVERRIDE { DIS, EN };
	enum class PIXELCLOCK { UNIFORM, NONUNIFORM };
	enum class RTCHAN { PIXELCLOCK, SCANGALVO, RESCANGALVO, DODEBUG, VISION, SCALINGVISION, FIDELITY, SCALINGFIDELITY, NCHAN };		//NCHAN = number of RT channels available including the channel for the pixelclock
	enum class FILTERWHEEL { DET, EXC };
	enum Axis { STAGEX, STAGEY, STAGEZ };
	enum class RUNMODE { SINGLE, LIVE, AVG, SCANZ, SCANZCENTERED, SCANXY, COLLECTLENS };
	enum class ACTION { CUT, ACQ, SAV, MOV };
	enum class LASER { VISION, FIDELITY, AUTO};
	enum class FILTERCOLOR { BLUE, GREEN, RED, OPEN, CLOSED };
	enum class COM { VISION = 1, FIDELITY = 8, FWDET = 5, FWEXC = 9, PMT16X = 6};
	enum ROIindices { YMIN = 0, XMIN = 1, YMAX = 2, XMAX = 3};
	enum class ZSCAN { BOTTOMUP = -1, TOPDOWN = 1};
	enum class PMT16XCHAN { CH00, CH01, CH02, CH03, CH04, CH05, CH06, CH07, CH08, CH09, CH10, CH11, CH12, CH13, CH14, CH15, CENTERED};

	extern const std::string folderPath;
	extern const std::string bitfilePath;
	extern const std::string openclFilePath;

	extern const INPUT photocounterInput;
	extern const PIXELCLOCK pixelclockType;

	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int sec;
	extern const int um;
	extern const int mm;
	extern const double mmps;
	extern const int V;
	extern const int mW;
	extern const double mWpum;

	extern const double VISIONpulsePeriod;
	extern const double pixelDwellTime;
	extern const double lineclockHalfPeriod;
	
	extern const int AOmax;
	extern const int tickPerUs;
	extern const double usPerTick;
	extern const U32 tMIN_tick;
	extern const int AO_tMIN;
	extern const int syncDOtoAO_tick;
	extern const int nPreframes;
	extern const double postsequenceTimer;
	extern const double linegateTimeout;
	extern const int FIFOtimeout_tick;
	extern const int FIFOINmax;

	extern const double pockelsFirstFrameDelay;
	extern const double pockelsSecondaryDelay;
	extern const double galvosCommonDelay;
	extern const double rescanGalvoDelay;

	extern const double stagePulseStretcher;
	extern const double	ZstageTrigDelayTopdown;
	extern const double	ZstageTrigDelayBottomup;

	extern const int nChanPMT;
	extern const int nPMTsim;
	extern const U8 PMTsimArray[];
}