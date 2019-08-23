#pragma once
#include <deque>
#include <vector>
#include <array>

#define multibeam 0			//Multibeam or singlebeam. *cast
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
	typedef std::deque<U32> QU32;			//Queue of unsigned integers
	typedef std::vector<QU32> VQU32;		//Vector of queues of unsigned integers

	//The scan directions are wrt the direction of motion of the stages
	//DOWNWARD: the stage z moves downward (the sample is scanned from bottom to top)
	//UPWARD: the stage z moves upward (the sample is scanned from top to bottom)
	//RIGHT: when facing the microscope, the stage x moves right (the sample is scanned from its right to its left)
	//LEFT: when facing the microscope, the stage x moves left (the sample is scanned from its left to its right)
	//OUTWARD: the stage y moves away from the optical table
	//INWARD: the stage y moves to the center of the optical table
	enum class SCANDIR { LEFTWARD, RIGHTWARD, OUTWARD, INWARD, DOWNWARD, UPWARD };
	struct SCANDIR3 { SCANDIR XX;  SCANDIR YY; SCANDIR ZZ; };
	struct INDICES2 { int II; int JJ; };
	struct POSITION2 { double XX; double YY; };
	struct POSITION3 { double XX; double YY; double ZZ; };
	struct VELOCITY3 { double XX; double YY; double ZZ; };
	struct FFOV2 { double XX; double YY; };
	struct LIMIT2 { double MIN; double MAX; };
	struct ROI4 { double YMIN; double XMIN; double YMAX;  double XMAX; };	//Region of interest
	struct SAMPLESIZE3 { double XX; double YY; double ZZ; };				//Sample size
	struct TILEOVERLAP4 { double XX; double YY; double ZZ; };				//Tile overlap fraction

	enum class PMTIN { PMT = false, SIM = true };							//*cast
	enum class LINECLOCK { RS = false, FG = true  };						//*cast
	enum class MAINTRIG { PC = 0, STAGEZ = 1, STAGEX = 2 };					//The numbering must match that of LV
	enum class FPGARESET { DIS = false, EN = true };
	enum class FIFOOUTfpga { DIS = false, EN = true  };						//*cast
	enum class TIFFSTRUCT { SINGLEPAGE, MULTIPAGE };
	enum class OVERRIDE { DIS, EN };
	enum class RUNMODE { SINGLE, LIVE, AVG, SCANZ, SCANZCENTERED, SCANXY, COLLECTLENS };
	enum class COM { VISION = 1, FIDELITY = 8, FWDET = 5, FWEXC = 9, PMT16X = 6};	//*cast

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
	extern const double g_pulsesPerPix;
	extern const U8 g_upscalingFactor;
	
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

	struct GALVOcalib {	double voltagePerDistance; double voltageOffset; };
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
	extern const U8 g_nPMTsim;
	extern const U8 g_PMTsimArray[];

	extern const double g_cLensPos750nm;
	extern const double g_cLensPos920nm;
	extern const double g_cLensPos1040nm;
}