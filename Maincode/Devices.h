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
	NiFpga_Status *status;
	NiFpga_Session session;
	//vibratome channels
	enum VibratomeChannel {
		VibratomeStart,
		VibratomeBack,
		VibratomeForward
	};
public:
	int Nslide;						//Slide number
	double sectionThickness;		//Thickness of the section
	double speed;					//Speed of the vibratome (manual setting)
	double amplitude;				//Amplitude of the vibratome (manual setting)
	Vibratome();
	~Vibratome();
	int setState();
	int sendCommand(double dt, VibratomeChannel channel);
};

class ResonantScanner
{
	NiFpga_Status *status;
	NiFpga_Session session;
	const int delayTime;
	double ResonantScanner::convertUm2Volt(double Amplitude);
public:
	bool state;
	double amplitude_um;
	double voltPerUm;		//Calibration factor. volts per microns
	ResonantScanner(NiFpga_Status* status, NiFpga_Session session);
	~ResonantScanner();
	int ResonantScanner::setState(bool state);
	int ResonantScanner::setOutputVoltage(double Vout);
	int ResonantScanner::setOutputAmplitude(double amplitude_um);
	int ResonantScanner::turnOn(double amplitude_um);
	int ResonantScanner::turnOff();
};

class Shutter
{
	NiFpga_Status *status;
	NiFpga_Session session;
	const int delayTime;
public:
	uint32_t IDshutter;
	bool state;
	Shutter(NiFpga_Status* status, NiFpga_Session session, uint32_t ID);
	~Shutter();
	int Shutter::setState(bool requestedState);
	int Shutter::pulseHigh();
};

class PixelClock
{
	U32Q Queue;
	const int latency_tick = 2;		//latency of detecting the line clock. Calibrate the latency with the oscilloscope. (C++11 allows initialization in declaration)
	double ConvertSpatialCoord2Time(double x);
	double getDiscreteTime(int pix);
	double calculateDwellTime(int pix);
	double calculatePracticalDwellTime(int pix);

public:
	PixelClock();
	~PixelClock();
	U32Q PixelClockEqualDuration();
	U32Q PixelClockEqualDistance();
};

