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

	enum class PMTIN { PMT = false, SIM = true };
	enum class LINECLOCK { RS = false, FG = true  };
	enum class MAINTRIG { PC = false, ZSTAGE = true };
	enum class FPGARESET { DIS = false, EN = true };
	enum class FIFOOUT { DIS = false, EN = true  };
	enum class MULTIPAGE { DIS = false, EN = true };
	enum class OVERRIDE { DIS = false, EN = true };
	enum class RUNMODE { SINGLE, LIVE, AVG, SCANZ, SCANZCENTERED, SCANXY, COLLECTLENS };
	enum class COM { VISION = 1, FIDELITY = 8, FWDET = 5, FWEXC = 9, PMT16X = 6};
	enum ROIindices { YMIN = 0, XMIN = 1, YMAX = 2, XMAX = 3};
	enum class ZSCAN { BOTTOMUP = -1, TOPDOWN = 1};

	extern const std::string folderPath;
	extern const std::string bitfilePath;
	extern const std::string openclFilePath;

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

	extern const double laserPulsePeriod;
	extern const double pixelDwellTime;
	extern const double lineclockHalfPeriod;
	
	extern const int AOmax;
	extern const int tickPerUs;
	extern const double usPerTick;
	extern const U32 tMIN_tick;
	extern const int AO_tMIN;
	extern const int syncDOtoAO_tick;
	extern const int nPreframes;

	extern const double linegateTimeout;
	extern const int FIFOtimeout_tick;
	extern const int FIFOINmax;

	extern const double pockelsFirstFrameDelay;
	extern const double pockelsSecondaryDelay;

	struct GALVOcalib
	{
		double voltagePerDistance;
		double voltageOffset;
	};
	extern const double scanGalvoDelay;
	extern const double rescanGalvoDelay;
	extern const GALVOcalib scannerCalib;
	extern const GALVOcalib rescannerCalibV750nm;
	extern const GALVOcalib rescannerCalibV920nm;
	extern const GALVOcalib rescannerCalibV1040nm;
	extern const GALVOcalib rescannerCalibF1040nm;
	extern const int PMT16Xchan_int;

	extern const double postsequenceTimer;

	extern const double	ZstageTrigDelayTopdown;
	extern const double	ZstageTrigDelayBottomup;

	extern const int nChanPMT;
	extern const PMTIN photocounterInput;
	extern const int nPMTsim;
	extern const U8 PMTsimArray[];

	extern const double cLensPos750nm;
	extern const double cLensPos920nm;
	extern const double cLensPos1040nm;
}