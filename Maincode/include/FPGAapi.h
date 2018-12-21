#pragma once
//#include <iostream>
//#include <string>		//std::to_string
#include <windows.h>	//Sleep. Also the PI stages
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include "Utilities.h"
using namespace Constants;

namespace FPGAns
{
	U16 convertTimeToTick(const double t);
	I16 convertVoltageToI16(const double voltage);
	double convertI16toVoltage(const int input);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double timeStep, const double AO);
	U32 packDigitalSinglet(const double timeStep, const bool DO);
	U32 packPixelclockSinglet(const double timeStep, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);
	void linearRamp(QU32 &queue, double timeStep, const double rampLength, const double Vi, const double Vf);

	//Establish a connection to the FPGA
	class FPGA
	{	
		NiFpga_Session mFpgaHandle;											//FPGA handle. Non-const to let the FPGA API assign the handle
		const std::string mBitfile = bitfilePath + NiFpga_FPGAvi_Bitfile;	//FPGA bitfile location

		void initializeFpga_() const;
	public:
		FPGA();
		~FPGA();
		void close(const FPGAresetSelector resetFlag = NORESET) const;
		NiFpga_Session getFpgaHandle() const;								//Access the handle indirectly to avoid modifying it by mistake

	};

	//Create a realtime control sequence and pixelclock
	class RTcontrol
	{
		//Private subclass
		class Pixelclock
		{
			QU32 mPixelclockQ;					//Queue containing the pixelclock
			const int mLatency_tick = 2;		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
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
		LineclockSelector mLineclockInput;															//Resonant scanner (RS) or Function generator (FG)
		AcqTriggerSelector mStageAsTrigger;															//Trigger the acquisition with the z stage: enable (0), disable (1)
		const double mDwell = 0.1625 * us;															//Dwell time = 13 * 12.5 ns = 162.5 ns (85 Mvps for 16X), Npix = 340
																									//Dwell time = 10 * 12.5 ns = 125 ns (128 Mvps for 16X), Npix = 400
		const double mPulsesPerPixel = mDwell / VISIONpulsePeriod;									//Max number of laser pulses per pixel
		const unsigned char mUpscaleU8 = static_cast<unsigned char>(255 / (mPulsesPerPixel + 1));	//Upscale the photoncount to cover the full 0-255 range of a 8-bit number. Plus one to avoid overflow
		const int mNlinesSkip = 0;																	//Number of lines to skip beetween frames to reduce the acquisition bandwidt
		int mWidthPerFrame_pix;																		//Width in pixels of a single frame (RS axis). I call each swing of the RS a "line"
		int mHeightPerFrame_pix;																	//Height in pixels of a single frame (galvo axis). This sets the number of "lines" in the image
		int mNframes;																				//Number of frames to acquire
		int mHeightAllFrames_pix;																	//Total number of lines in all the frames without including the skipped lines
		int mNpixAllFrames;																			//Total number of pixels in all the frames (the skipped lines don't acquire pixels)

		RTcontrol(const FPGAns::FPGA &fpga, const LineclockSelector lineclockInput = FG,
			const int nFrames = 1, const int widthPerFrame_pix = 300, const int heightPerFrame_pix = 400, const AcqTriggerSelector mStageAsTrigger = PCTRIG);
		RTcontrol(const RTcontrol&) = delete;				//Disable copy-constructor
		RTcontrol& operator=(const RTcontrol&) = delete;	//Disable assignment-constructor
		RTcontrol(RTcontrol&&) = delete;					//Disable move constructor
		RTcontrol& operator=(RTcontrol&&) = delete;			//Disable move-assignment constructor

		void pushQueue(const RTchannel chan, QU32& queue);
		void clearQueue(const RTchannel chan);
		void pushDigitalSinglet(const RTchannel chan, double timeStep, const bool DO);
		void pushAnalogSinglet(const RTchannel chan, double timeStep, const double AO, const OverrideFileSelector overrideFlag = NOOVERRIDE);
		void pushAnalogSingletFx2p14(const RTchannel chan, const double scalingFactor);
		void pushLinearRamp(const RTchannel chan, double timeStep, const double rampLength, const double Vi, const double Vf);
		void presetFPGAoutput() const;
		void uploadRT() const;
		void triggerRT() const;
		void setZstageTriggerEnabled(const bool state);
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}