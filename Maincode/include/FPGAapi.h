#pragma once
#include <windows.h>	//Sleep. Also the PI stages
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include "Utilities.h"
using namespace Constants;

namespace FPGAns
{
	U16 timeToTick(const double t);
	I16 voltageToI16(const double voltage);
	double I16toVoltage(const int input);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double timeStep, const double AO);
	U32 packDigitalSinglet(const double timeStep, const bool DO);
	U32 packPixelclockSinglet(const double timeStep, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);
	void linearRamp(QU32 &queue, double timeStep, const double rampLength, const double Vi, const double Vf);

	//Establish communication to the FPGA
	class FPGA
	{	
		NiFpga_Session mHandle;												//FPGA handle. Non-const to let the FPGA API assign the handle
		const std::string mBitfile{ bitfilePath + NiFpga_FPGAvi_Bitfile };	//FPGA bitfile location

		void initializeFpga_() const;
	public:
		FPGA();
		~FPGA();
		void close(const FPGARESET reset = FPGARESET::DIS) const;
		NiFpga_Session getHandle() const;									//Access the handle indirectly to avoid modifying it by mistake

	};

	//Create a realtime control sequence and pixelclock
	class RTcontrol
	{
		//Private subclass
		class Pixelclock
		{
			QU32 mPixelclockQ;					//Queue containing the pixelclock
			const int mLatency_tick{ 2 };		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
			double mDwell;
			int mWidthPerFrame_pix;

			void pushUniformDwellTimes(const int calibFine_tick);
		public:
			Pixelclock(const int widthPerFrame_pix, const double dwell);
			QU32 readPixelclock() const;
		};

		VQU32 mVectorOfQueues;

		void concatenateQueues_(QU32& receivingQueue, QU32& givingQueue) const;
		void uploadImagingParameters_() const;
		void uploadFIFOIN_(const VQU32 &vectorOfQueues) const;
		void triggerNRT_() const;

	public:
		const FPGAns::FPGA &mFpga;
		LINECLOCK mLineclockInput;														//Resonant scanner (RS) or Function generator (FG)
		MAINTRIG mMainTrigger;															//Trigger the acquisition with the z stage: enable (0), disable (1)
		FIFOOUT mFIFOOUTstate;															//Enable or disable the fpga FIFOOUT
		const double mDwell{ 0.1625 * us };												//mDwell = 13 * 12.5 ns = 162.5 ns
		const double mPulsesPerPix = mDwell / VISIONpulsePeriod;						//Max number of laser pulses per pixel
		const U8 mUpscaleFactorU8{ static_cast<U8>(255 / (mPulsesPerPix + 1)) };		//Upscale 4-bit counts to 8-bit (range 0-255) for compatibility with ImageJ's standards. Plus one to avoid overflow
		int mWidthPerFrame_pix;															//Width in pixels of a single frame (RS axis). I call each swing of the RS a "line"
		int mHeightPerFrame_pix;														//Height in pixels of a single frame (galvo axis). This sets the number of "lines" in the image
		int mNframes;																	//Number of frames to acquire
		int mHeightAllFrames_pix;														//Total number of lines in all the frames without including the skipped lines
		int mNpixAllFrames;																//Total number of pixels in all the frames (the skipped lines don't acquire pixels)
		PMT16XCHAN mPMT16Xchan;															//PMT16X channel to be used

		RTcontrol(const FPGAns::FPGA &fpga, const LINECLOCK lineclockInput , const MAINTRIG mainTrigger,
			const int nFrames, const int widthPerFrame_pix, const int heightPerFrame_pix, FIFOOUT FIFOOUTstate, PMT16XCHAN PMT16Xchan);
		RTcontrol(const FPGAns::FPGA &fpga);
		RTcontrol(const RTcontrol&) = delete;				//Disable copy-constructor
		RTcontrol& operator=(const RTcontrol&) = delete;	//Disable assignment-constructor
		RTcontrol(RTcontrol&&) = delete;					//Disable move constructor
		RTcontrol& operator=(RTcontrol&&) = delete;			//Disable move-assignment constructor

		void pushQueue(const RTCHAN chan, QU32& queue);
		void clearQueue(const RTCHAN chan);
		void pushDigitalSinglet(const RTCHAN chan, double timeStep, const bool DO);
		void pushAnalogSinglet(const RTCHAN chan, double timeStep, const double AO, const OVERRIDE override = OVERRIDE::DIS);
		void pushAnalogSingletFx2p14(const RTCHAN chan, const double scalingFactor);
		void pushLinearRamp(const RTCHAN chan, double timeStep, const double rampLength, const double Vi, const double Vf);
		void presetFPGAoutput() const;
		void uploadRT() const;
		void triggerRT() const;
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}