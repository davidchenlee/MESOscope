#pragma once
#include <deque>
#include <vector>
#include <array>
#define g_multibeam 0						//Multibeam or singlebeam. *cast
#define g_pockelsAutoOff 1					//For debugging purposes. In LV, let Framegate gate the output of the pockels

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
	//DOWNWARD: the stage Z moves downward (the sample is scanned from bottom to top)
	//UPWARD: the stage Z moves upward (the sample is scanned from top to bottom)
	//RIGHT: when facing the microscope, the stage X moves right (the sample is scanned from its right to its left)
	//LEFT: when facing the microscope, the stage X moves left (the sample is scanned from its left to its right)
	//OUTWARD: the stage Y moves away from the optical table
	//INWARD: the stage Y moves to the center of the optical table
	enum class SCANDIR { LEFTWARD, RIGHTWARD, OUTWARD, INWARD, DOWNWARD, UPWARD };
	struct SCANDIR3 { SCANDIR XX;  SCANDIR YY; SCANDIR ZZ; };
	enum AXIS { XX, YY, ZZ };

	struct POSITION2 { double XX; double YY; };
	struct POSITION3 { double XX; double YY; double ZZ; };
	struct LENGTH2 { double XX; double YY; };								//Size in 2D
	struct LENGTH3 { double XX; double YY; double ZZ; };					//Size in 3D
	struct VELOCITY3 { double XX; double YY; double ZZ; };
	struct FFOV2 { double XX; double YY; };
	struct LIMIT2 { double MIN; double MAX; };
	struct ROI4 { double YMIN; double XMIN; double YMAX;  double XMAX; };	//Region of interest
	struct TILEIJ { int II; int JJ; };										//Tile array indices IJ
	struct TILEDIM2 { int II; int JJ; };									//Tile array dimension in 2D
	struct TILEOVERLAP3 { double II; double JJ; double KK; };				//Tile overlap fraction
	struct PIXELij { int ii; int jj; };										//Pixel indices ij
	struct PIXDIM2 { int ii; int jj; };										//Dimension in 2D pixels

	enum class PMTIN { PMT = false, SIM = true };							//*cast
	enum class LINECLOCK { RS = false, FG = true  };						//*cast
	enum class MAINTRIG { PC = 0, STAGEZ = 1, STAGEX = 2 };					//IMPORTANT! The number assignmnent must match that of LV
	enum class FPGARESET { DIS = false, EN = true };
	enum class FIFOOUTfpga { DIS = false, EN = true  };						//*cast
	enum class TIFFSTRUCT { SINGLEPAGE, MULTIPAGE };
	enum class OVERRIDE { DIS, EN };
	enum class RUNMODE { SINGLE, LIVE, AVG, SCANZ, SCANZCENTERED, SCANX, COLLECTLENS, FIELD_ILLUM };
	enum class COM { VISION = 1, FIDELITY = 8, FWDET = 5, FWEXC = 9, PMT16X = 6};	//*cast
	enum class RUN { DIS = false, EN = true };

	extern const std::string g_imagingFolderPath;
	extern const std::string g_bitfilePath;
	extern const std::string g_openclFilePath;

	extern const double PI;
	extern const int us;
	extern const int ms;
	extern const int seconds;
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
	extern const double g_scannerDelay;
	extern const double g_rescannerDelay;
	extern const GALVOcalib g_scannerCalib;
	extern const GALVOcalib g_rescannerCalibV750nm;
	extern const GALVOcalib g_rescannerCalibV920nm;
	extern const GALVOcalib g_rescannerCalibV1040nm;
	extern const GALVOcalib g_rescannerCalibF1040nm;
	extern const int g_rescanner1Xchan_int;

	extern const SCANDIR3 g_initialStageScanDirXYZ;
	extern const double	g_STAGEZtrigAcqDelay750nm;
	extern const double	g_STAGEZtrigAcqDelay1040nm;
	extern const double	g_STAGEXtrigAcqDelay;
	extern const POSITION3 g_chromaticShiftVision750nm;
	extern const POSITION3 g_chromaticShiftVision920nm;
	extern const POSITION3 g_chromaticShiftVision1040nm;
	extern const POSITION3 g_chromaticShiftFidelity1040nm;

	extern const int g_nChanPMT;
	extern const PMTIN g_photocounterInput;
	extern const U8 g_nPMTsim;
	extern const U8 g_PMTsimArray[];

	extern const double g_cLensPos750nm;
	extern const double g_cLensPos920nm;
	extern const double g_cLensPos1040nm;
}