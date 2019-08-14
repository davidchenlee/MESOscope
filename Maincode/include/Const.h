#pragma once
#include <deque>
#include <vector>
#include <array>

#define multibeam 0			//Multibeam or singlebeam
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

	enum class PMTIN { PMT = false, SIM = true };//casted
	enum class LINECLOCK { RS = false, FG = true  };//casted
	enum class MAINTRIG { PC = 0, STAGEZ = 1, STAGEX = 2 };	//The numbering must match that of LV
	enum class FPGARESET { DIS = false, EN = true };//casted
	enum class FIFOOUT { DIS = false, EN = true  };//casted
	enum class TIFFSTRUCT { SINGLEPAGE, MULTIPAGE };
	enum class OVERRIDE { DIS, EN };
	enum class RUNMODE { SINGLE, LIVE, AVG, SCANZ, SCANZCENTERED, SCANXY, COLLECTLENS };
	enum class COM { VISION = 1, FIDELITY = 8, FWDET = 5, FWEXC = 9, PMT16X = 6};
	enum ROIindices { YMIN = 0, XMIN = 1, YMAX = 2, XMAX = 3};
	enum class ZSCAN { BOTTOMUP = -1, TOPDOWN = 1};//Direction of imaging (TOPDOWN = stage z moves up. BOTTOMUP = stage z moves down)
	enum class SCANDIRX { NEG = -1, POS = 1};

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

	extern const double g_laserPulsePeriod;
	extern const double g_pixelDwellTime;
	extern const double g_lineclockHalfPeriod;
	
	extern const int g_AOmax;
	extern const int g_tickPerUs;
	extern const double g_usPerTick;
	extern const U32 g_tMin_tick;
	extern const int g_tMinAO;
	extern const int g_DOdelay_tick;
	extern const int g_nPreframes;
	extern const double g_linegateTimeout;
	extern const double g_postSequenceTimer;
	extern const double g_stageDebounceTimer;
	extern const int g_FIFOtimeout_tick;
	extern const int g_FIFOINmax;

	extern const double g_pockelsFirstFrameDelay;
	extern const double g_pockelsSecondaryDelay;

	struct GALVOcalib
	{
		double voltagePerDistance;
		double voltageOffset;
	};
	extern const double g_scanGalvoDelay;
	extern const double g_rescanGalvoDelay;
	extern const GALVOcalib g_scannerCalib;
	extern const GALVOcalib g_rescannerCalibV750nm;
	extern const GALVOcalib g_rescannerCalibV920nm;
	extern const GALVOcalib g_rescannerCalibV1040nm;
	extern const GALVOcalib g_rescannerCalibF1040nm;
	extern const int g_PMT16Xchan_int;

	extern const double	g_STAGEZtrigAcqDelayTopdown;
	extern const double	g_STAGEZTrigAcqDelayBottomup;
	extern const double	g_STAGEXTrigAcqDelay;

	extern const int g_nChanPMT;
	extern const PMTIN g_photocounterInput;
	extern const int g_nPMTsim;
	extern const U8 g_PMTsimArray[];

	extern const double g_cLensPos750nm;
	extern const double g_cLensPos920nm;
	extern const double g_cLensPos1040nm;
}