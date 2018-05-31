#pragma once
#include <iostream>
#include <string>		//For std::to_string
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

	U16 convertUsTotick(const double t_us);
	I16 convertVoltToI16(const double voltage_V);
	U32 packU32(const U16 t_tick, const U16 AO_U16);
	U32 packAnalogSinglet(const double t_us, const double AO_V);
	U32 packDigitalSinglet(const double t_us, const bool DO);
	U32 packPixelclockSinglet(const double t_us, const bool DO);
	void checkStatus(char functionName[], NiFpga_Status status);

	class Session
	{
		NiFpga_Session mSession;
	public:
		Session();
		~Session();
		void initialize() const;
		void writeFIFOpc(VQU32 &vectorqueues) const;
		void triggerRT() const;
		void enableFIFOfpga() const;
		void flushBRAMs() const;
		void close(const bool reset) const;
		NiFpga_Session getSession() const;
	};

	class RTsequence
	{
		class Pixelclock
		{
			QU32 pixelclockQ;					//Queue containing the pixel-clock sequence
			const int mLatency_tick = 2;		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
			double convertSpatialCoordToTime_us(const double x) const;
			double getDiscreteTime_us(const int pix) const;
			double calculateDwellTime_us(const int pix) const;
			double calculatePracticalDwellTime_us(const int pix) const;
			void pushUniformDwellTimes();
			void pushCorrectedDwellTimes();
		public:
			Pixelclock();
			~Pixelclock();
			QU32 readPixelclock() const;
		};

		const FPGAapi::Session &mFpga;
		VQU32 mVectorOfQueues;
		void concatenateQueues(QU32& receivingQueue, QU32& givingQueue);
	public:
		RTsequence(const FPGAapi::Session &fpga);
		RTsequence(const RTsequence&) = delete;				//Disable copy-constructor
		RTsequence& operator=(const RTsequence&) = delete;	//Disable assignment-constructor
		~RTsequence();
		void pushQueue(const RTchannel chan, QU32& queue);
		void clearQueue(const RTchannel chan);
		void pushDigitalSinglet(const RTchannel chan, double t_us, const bool DO);
		void pushAnalogSinglet(const RTchannel chan, double t_us, const double AO_V);
		void pushAnalogSingletFx2p16(const RTchannel chan, const double AO_fx2p14);
		void pushLinearRamp(const RTchannel chan, double timeStep_us, const double rampLength, const double Vinitial, const double Vfinal);
		void uploadRT();
		void triggerRT();
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}