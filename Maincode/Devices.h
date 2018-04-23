#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
#include "UARTscope.h"
#include "Tiffscope.h"


//Photon counter
int readPhotonCount(NiFpga_Status* status, NiFpga_Session session);
void readFIFO(NiFpga_Status* status, NiFpga_Session session, int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray);
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrameToTxt(unsigned char *imageArray, std::string fileName);

//Vibratome functions
class Vibratome
{
	NiFpga_Session mSession;
	//vibratome channels
	enum VibratomeChannel {
		VibratomeStart,
		VibratomeBack,
		VibratomeForward
	};
public:
	int mNslide;						//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
	Vibratome();
	~Vibratome();
	NiFpga_Status startStop();
	NiFpga_Status sendCommand(double dt, VibratomeChannel channel);
};

class ResonantScanner
{
	NiFpga_Session mSession;
	const int mDelayTime = 10;
	double ResonantScanner::convertUm2Volt(double Amplitude);
public:
	bool mState;							//is the scanner on or off
	double mAmplitude_um = 0;
	double mVoltPerUm = RS_voltPerUm;		//Calibration factor. volts per microns
	ResonantScanner(NiFpga_Session session);
	~ResonantScanner();
	NiFpga_Status ResonantScanner::startStop(bool requestedState);
	NiFpga_Status ResonantScanner::setOutputVoltage(double Vout);
	NiFpga_Status ResonantScanner::setOutputAmplitude(double amplitude_um);
	NiFpga_Status ResonantScanner::turnOn(double amplitude_um);
	NiFpga_Status ResonantScanner::turnOff();
};

class Shutter
{
	NiFpga_Session mSession;
	const int mDelayTime = 10;
public:
	uint32_t mIDshutter;
	bool mState;
	Shutter(NiFpga_Session session, uint32_t ID);
	~Shutter();
	NiFpga_Status Shutter::setOutput(bool requestedState);
	NiFpga_Status Shutter::pulseHigh();
};

class PixelClock
{
	QU32 Queue;
	const int latency_tick = 2;		//latency of detecting the line clock. Calibrate the latency with the oscilloscope. (C++11 allows initialization in declaration)
	double ConvertSpatialCoord2Time(double x);
	double getDiscreteTime(int pix);
	double calculateDwellTime(int pix);
	double calculatePracticalDwellTime(int pix);

public:
	PixelClock();
	~PixelClock();
	QU32 PixelClockEqualDuration();
	QU32 PixelClockEqualDistance();
};

class Stage
{
public:
	Stage();
	~Stage();
};


