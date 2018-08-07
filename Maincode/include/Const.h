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
	typedef std::array<double, 3> double3;		//array of 3 doubles
	typedef std::array<int, 3> int3;			//array of 3 ints

	enum PhotonCounterInputSelector { ext, sim };
	enum LineclockInputSelector { RS, FG };
	enum PixelclockSelector { uniform, nonuniform };
	enum RTchannel { PIXELCLOCK, GALVO1, GALVO2, DOdebug, POCKELS1, SCALING1, nChan };		//nChan = number of RT channels available, including the pixel clock channel
	enum ShutterID { Shutter1 = 1, Shutter2 };
	enum FilterwheelID { FW1 = 1, FW2 };
	enum Filtercolor { BLUE = 1, GREEN, RED };
	enum Axis { xx, yy, zz };
	enum RunMode { single, continuous, average, stack, stack_centered };

	enum ComID { VISIONcom, FW1com , FW2com, mPMTcom, ZSTAGEcom };
	extern const std::vector<std::string> assignCOM;

	extern const std::string folderPath;
	extern const std::string bitfilePath;

	extern const LineclockInputSelector lineclockInput;
	extern const PhotonCounterInputSelector photoncounterInput;
	extern const PixelclockSelector pixelclockType;
	extern const bool FIFOOUTfpgaEnable;
	extern const bool pockels1_enableAutoOff;

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
	extern const size_t FIFOINmax;

	extern const int stageTriggerPulse_ms;

	extern const int nPulses;
	extern const U8 pulseArray[];

	extern double mDwell_us;									//Dwell time = 13 * 12.5 ns = 162.5 ns (85 Mvps for 16X), Npix = 340. Dwell time = 10 * 12.5 ns = 125 ns (128 Mvps for 16X), Npix = 400
	extern double mPulsesPerPixel;			//Max number of laser pulses per pixel
	extern double mUpscaleU8;				//Upscale the photoncount to cover the full 0-255 range of a 8-bit number. Plus one to avoid overflow
	extern int mWidthPerFrame_pix;									//Width in pixels of a frame (RS axis). I call each swing of the RS a "line"
	extern int mHeightPerFrame_pix;									//Height in pixels of a frame (galvo axis). This sets the number of "lines" in the image
	extern int mNlinesSkip;											//Number of lines to skip beetween frames to reduce the acquisition bandwidt
	extern int mNframes;												//Number of frames to acquire
	extern int mHeightAllFrames_pix;		//Total number of lines in all the frames without including the skipped lines
	extern int mNpixAllFrames;	//Total number of pixels in all the frames (the skipped lines don't acquire pixels)
}