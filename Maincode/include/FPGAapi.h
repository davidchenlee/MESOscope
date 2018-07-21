#pragma once
//#include <iostream>
//#include <string>		//For std::to_string
#include <windows.h>	//the stages use this lib. also Sleep
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include "Utilities.h"
using namespace Constants;
using namespace Parameters;

namespace FPGAapi
{
	/*Define the full path of the bitfile. The bitfile is the code that runs on the FPGA*/
	static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

	U16 convertUsTotick(const double t);
	I16 convertVoltToI16(const double voltage_V);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double timeStep, const double AO_V);
	U32 packDigitalSinglet(const double timeStep, const bool DO);
	U32 packPixelclockSinglet(const double timeStep, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);

	class Session
	{
		NiFpga_Session mSession;
	public:
		Session();
		~Session();
		void initialize() const;
		void writeFIFOpc(const VQU32 &vectorqueues) const;
		void triggerRT() const;
		void flushBRAMs() const;
		void close(const bool reset) const;
		NiFpga_Session getSession() const;
	};

	class RTsequence
	{
		class Pixelclock
		{
			QU32 mPixelclockQ;					//Queue containing the pixel-clock sequence
			const int mLatency_tick = 2;		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
			void pushUniformDwellTimes(const int calibFine_tick, const double dwellTime_us);
		public:
			Pixelclock();
			~Pixelclock();
			QU32 readPixelclock() const;
		};

		const FPGAapi::Session &mFpga;
		VQU32 mVectorOfQueues;
		void concatenateQueues(QU32& receivingQueue, QU32& givingQueue) const;
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
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}