#pragma once
//#include <iostream>
//#include <string>		//std::to_string
#include <windows.h>	//Sleep. Also the PI stages
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include "Utilities.h"
using namespace Constants;

namespace FPGAapi
{
	U16 convertUsTotick(const double t);
	I16 convertVoltToI16(const double voltage_V);
	double convertI16toVolt(const int input);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double timeStep, const double AO_V);
	U32 packDigitalSinglet(const double timeStep, const bool DO);
	U32 packPixelclockSinglet(const double timeStep, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);

	//Establish a connection with the FPGA
	class Session
	{	
		NiFpga_Session mFpgaHandle;											//FPGA handle
		const std::string mBitfile = bitfilePath + NiFpga_FPGAvi_Bitfile;	//FPGA bitfile location

		void FIFOOUTpcGarbageCollector_() const;
		void flushBRAMs_() const;
	public:
		double mDwell_us = 0.1625 * us;									//Dwell time = 13 * 12.5 ns = 162.5 ns (85 Mvps for 16X), Npix = 340. Dwell time = 10 * 12.5 ns = 125 ns (128 Mvps for 16X), Npix = 400
		double mPulsesPerPixel = mDwell_us / VISIONpulsePeriod;			//Max number of laser pulses per pixel
		double mUpscaleU8 = 255 / (mPulsesPerPixel + 1);				//Upscale the photoncount to cover the full 0-255 range of a 8-bit number. Plus one to avoid overflow
		int mWidthPerFrame_pix = 300;									//Width in pixels of a frame (RS axis). I call each swing of the RS a "line"
		int mHeightPerFrame_pix = 400;									//Height in pixels of a frame (galvo axis). This sets the number of "lines" in the image
		int mNlinesSkip = 0;											//Number of lines to skip beetween frames to reduce the acquisition bandwidt
		int mNframes = 1;												//Number of frames to acquire
		int mHeightAllFrames_pix = mHeightPerFrame_pix * mNframes;		//Total number of lines in all the frames without including the skipped lines
		int mNpixAllFrames = mWidthPerFrame_pix * mHeightAllFrames_pix;	//Total number of pixels in all the frames (the skipped lines don't acquire pixels)

		Session();
		~Session();
		void initialize() const;
		void writeFIFOINpc(const VQU32 &vectorqueues) const;
		void triggerRT() const;
		void close(const bool reset) const;
		NiFpga_Session getSession() const;
	};

	//Create a realtime sequence and pixelclock
	class RTsequence
	{
		const FPGAapi::Session &mFpga;
		VQU32 mVectorOfQueues;

		void concatenateQueues_(QU32& receivingQueue, QU32& givingQueue) const;

		//Private subclass
		class Pixelclock
		{
			const FPGAapi::Session &mFpga;
			QU32 mPixelclockQ;					//Queue containing the pixel-clock sequence
			const int mLatency_tick = 2;		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
			void pushUniformDwellTimes(const int calibFine_tick, const double dwellTime_us);
		public:
			Pixelclock(const FPGAapi::Session &fpga);
			~Pixelclock();
			QU32 readPixelclock() const;
		};

	public:
		RTsequence(const FPGAapi::Session &fpga);
		RTsequence(const RTsequence&) = delete;				//Disable copy-constructor
		RTsequence& operator=(const RTsequence&) = delete;	//Disable assignment-constructor
		~RTsequence();
		void pushQueue(const RTchannel chan, QU32& queue);
		void clearQueue(const RTchannel chan);
		void pushDigitalSinglet(const RTchannel chan, double timeStep, const bool DO);
		void pushAnalogSinglet(const RTchannel chan, double timeStep, const double AO_V);
		void pushAnalogSingletFx2p14(const RTchannel chan, const double scalingFactor);
		void pushLinearRamp(const RTchannel chan, double timeStep, const double rampLength, const double Vi_V, const double Vf_V);
		void uploadRT() const;
		void triggerRT() const;
		FPGAapi::Session getSession() const;
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}