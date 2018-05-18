#pragma once
#include <iostream>
#include <string>	//For std::to_string
#include "NiFpga_FPGAvi.h"
#include "Const.h"
using namespace Constants;
using namespace Parameters;


/*Define the full path of the bitfile. The bitfile is the FPGA code*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

namespace FPGAapi
{
	U16 convertUs2tick(const double t_us);
	I16 convertVolt2I16(const double voltage_V);
	U32 packU32(const U16 t_tick, const U16 val);
	U32 packAnalogSinglet(const double t_us, const double val);
	U32 packDigitalSinglet(const double t_us, const bool DO);
	U32 packPixelclockSinglet(const double t_us, const bool DO);

	class FPGAsession
	{
		NiFpga_Session mSession;
	public:
		FPGAsession();
		~FPGAsession();
		void initialize() const;
		void writeFIFO(VQU32 &vectorqueues) const;
		void triggerRT() const;
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
			double ConvertSpatialCoord2Time_us(const double x);
			double getDiscreteTime_us(const int pix);
			double calculateDwellTime_us(const int pix);
			double calculatePracticalDwellTime_us(const int pix);
			void uniformDwellTimes();
			void correctedDwellTimes();
		public:
			Pixelclock();
			~Pixelclock();
			QU32 readPixelclock() const;
		};

		const FPGAapi::FPGAsession &mFpga;
		VQU32 mVectorOfQueues;
		void concatenateQueues(QU32& receivingQueue, QU32& givingQueue);
	public:
		RTsequence(const FPGAapi::FPGAsession &fpga);
		RTsequence(const RTsequence&) = delete;				//Disable copy-constructor
		RTsequence& operator=(const RTsequence&) = delete;	//Disable assignment-constructor
		~RTsequence();
		void pushQueue(const RTchannel chan, QU32& queue);
		void pushDigitalSinglet(const RTchannel chan, double t_us, const bool DO);
		void pushAnalogSinglet(const RTchannel chan, const double t_us, const double AO);
		void pushLinearRamp(const RTchannel chan, double TimeStep, const double RampLength, const double Vinitial, const double Vfinal);
		void uploadRT();
	};

	class FPGAexception : public std::runtime_error
	{
	public:
		FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
	};
}


void checkFPGAstatus(char functionName[], NiFpga_Status status);



