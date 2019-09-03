#pragma once
#include <windows.h>				//Sleep. Also the PI stages
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include "Utilities.h"
#include <conio.h>					//For _getch()
using namespace Constants;

namespace FPGAfunc
{
	U16 timeToTick(const double t);
	I16 voltageToI16(const double voltage);
	double intToVoltage(const int input);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double timeStep, const double AO);
	U32 packDigitalSinglet(const double timeStep, const bool DO);
	U32 packPixelclockSinglet(const double timeStep, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);
	void linearRamp(QU32 &queue, double timeStep, const double rampLength, const double Vi, const double Vf);
	void concatenateQueues(QU32& receivingQueue, QU32& givingQueue);
}

class FPGA
{
public:
	FPGA();
	~FPGA();

	NiFpga_Session handle() const;										//Access the handle indirectly to avoid modifying it by mistake
	void close(const FPGARESET reset = FPGARESET::DIS) const;

	void setLineclock(const LINECLOCK lineclockInput) const;
	void setMainTrig(const MAINTRIG mainTrigger) const;
	void setStageTrigDelay(const MAINTRIG mainTrigger, const int heightPerBeamletPerFrame_pix, const SCANDIR scanDir) const;
	void enableFIFOOUTfpga(const FIFOOUTfpga enableFIFOOUTfpga) const;
	void enablePockelsScaling() const;

	I16 readScannerVoltageMon() const;
	I16 readRescannerVoltageMon() const;
	void uploadImagingParameters(const int mHeightPerBeamletAllFrames_pix, const int mHeightPerBeamletPerFrame_pix, const int mNframes) const;

	void triggerAOext() const;
	void triggerControlSequence() const;

	void startFIFOOUTpc() const;
	void stopFIFOOUTpc() const;
	void configureFIFOOUTpc(const U32 depth) const;
	void collectFIFOOUTpcGarbage() const;

	void uploadFIFOIN(const VQU32 &queue_vec, const U8 nChan) const;
	void readFIFOOUTpc(const int &nPixPerBeamletAllFrames, U32 *mBufferA, U32 *mBufferB) const;
private:
	NiFpga_Session mHandle;												//FPGA handle. Non-const to let the FPGA API assign the handle
	const std::string mBitfile{ bitfilePath + NiFpga_FPGAvi_Bitfile };	//FPGA bitfile location

	void initializeFpga_() const;
	void readChunk_(const int &nPixPerBeamletAllFrames, int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 &FIFOOUTpc, U32* buffer, int &timeout) const;
};

class RTcontrol
{
public:
	enum class RTCHAN { PIXELCLOCK, SCANNER, RESCANNER, DODEBUG, VISION, SCALINGVISION, FIDELITY, SCALINGFIDELITY, NCHAN };				//NCHAN = number of sequence channels available including the channel for the pixelclock
	enum class PMT16XCHAN { CH00, CH01, CH02, CH03, CH04, CH05, CH06, CH07, CH08, CH09, CH10, CH11, CH12, CH13, CH14, CH15, CENTERED };	//*cast but not relevant, only for debugging
	const FPGA &mFpga;
	SCANDIR mScanDir{ SCANDIR::UPWARD };				//Scan direction of the stage for continuous scan
	const PMT16XCHAN mPMT16Xchan;						//PMT16X channel to be used
	const int mWidthPerFrame_pix;						//Width in pixels of a single frame (fast axis). I call each swing of the RS a "line"
	const int mHeightPerBeamletPerFrame_pix;			//Height in pixels of a single beamlet in a single frame (slow axis)
	int mNframes;										//Number of frames to acquire
	int mHeightPerBeamletAllFrames_pix;					//Total number of lines per beamlet in all the frames
	int mNpixPerBeamletAllFrames;						//Total number of pixels per beamlet in all the frames

	RTcontrol(const FPGA &fpga, const LINECLOCK lineclockInput, const MAINTRIG mainTrigger, const FIFOOUTfpga enableFIFOOUTfpga, const int heightPerBeamletPerFrame_pix, const int widthPerFrame_pix, const int nFrames);
	RTcontrol(const FPGA &fpga, const LINECLOCK lineclockInput, const MAINTRIG mainTrigger, const FIFOOUTfpga enableFIFOOUTfpga, const int heightPerBeamletPerFrame_pix, const int widthPerFrame_pix);
	~RTcontrol();
	RTcontrol(const RTcontrol&) = delete;				//Disable copy-constructor
	RTcontrol& operator=(const RTcontrol&) = delete;	//Disable assignment-constructor
	RTcontrol(RTcontrol&&) = delete;					//Disable move constructor
	RTcontrol& operator=(RTcontrol&&) = delete;			//Disable move-assignment constructor

	void pushQueue(const RTCHAN chan, QU32& queue);
	void clearQueue(const RTCHAN chan);
	void pushDigitalSinglet(const RTCHAN chan, double timeStep, const bool DO);
	void pushAnalogSinglet(const RTCHAN chan, double timeStep, const double AO, const OVERRIDE override = OVERRIDE::DIS);
	void pushAnalogSingletFx2p14(const RTCHAN chan, const double scalingFactor);
	void pushLinearRamp(const RTCHAN chan, double timeStep, const double rampLength, const double Vi, const double Vf, const OVERRIDE override);

	void setNumberOfFrames(const int nFrames);
	void initialize(const SCANDIR scanDirZ = SCANDIR::UPWARD);
	void run();
	void downloadData();
	U32* dataBufferA() const;
	U32* dataBufferB() const;
private:
	class Pixelclock
	{
	public:
		Pixelclock(const int widthPerFrame_pix, const double dwell);
		QU32 readPixelclock() const;
	private:
		QU32 mPixelclockQ;					//Queue containing the pixelclock
		const int mLatency_tick{ 2 };		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
		double mDwell;
		int mWidthPerFrame_pix;
		const int mCalibFine_tick{ -40 };	//Fine tune the relative delay of the pixel clock wrt the line clock. Acquire an averaged image
											//and align the pixels corresponding to forward and backward scans of the RS
		void pushUniformDwellTimes_();
	};

	const LINECLOCK mLineclockInput;		//Resonant scanner (RS) or Function generator (FG)
	const MAINTRIG mMainTrigger;					//Trigger the acquisition with the Z-stage: enable (0), disable (1)
	const FIFOOUTfpga mEnableFIFOOUTfpga;			//Enable or disable the FIFOOUTfpga on the FPGA
	VQU32 mVec_queue;
	U32* mBufferA{ nullptr };				//Buffer array to read FIFOOUTpc A
	U32* mBufferB{ nullptr };				//Buffer array to read FIFOOUTpc B

	PMT16XCHAN determineRescannerSetpoint_() const;
	void presetScannerPosition_() const;
	void initializeStages_(const SCANDIR stackScanDir);
	void uploadControlSequence_() const;
	void correctInterleaved_();
};

class FPGAexception : public std::runtime_error
{
public:
	FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
};

class DataDownloadException : public std::runtime_error
{
public:
	DataDownloadException(const std::string& message) : std::runtime_error(message.c_str()) {}
};