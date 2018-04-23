#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
#include "UARTscope.h"
#include "Tiffscope.h"
#include "windows.h"	//the stages use this lib. also Sleep
#include <fstream>      //file management
#include <ctime>		//Clock()


//Genetic functions
void printHex(int input);
U32 packU32(U16 t, U16 x);
U16 convertUs2tick(double x);
I16 convertVolt2I16(double x);
U32 singleAnalogOut(double t, double V);
U32 singleDigitalOut(double t, bool DO);
U32 singlePixelClock(double t, bool DO);
QU32 generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal);
void concatenateQueues(QU32& headQ, QU32 tailQ);


//Image handling
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrameToTxt(unsigned char *imageArray, std::string fileName);


class PhotonCounter
{
	FPGAapi mFpga;
public:
	PhotonCounter(FPGAapi fpga);
	~PhotonCounter();
	NiFpga_Status readPhotonCount();
	NiFpga_Status readFIFO(int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray);
	NiFpga_Status configureFIFO(U32 depth);
};


class Vibratome
{
	FPGAapi mFpga;

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

	Vibratome(FPGAapi fpga);
	~Vibratome();
	NiFpga_Status startStop();
	NiFpga_Status sendCommand(double dt, VibratomeChannel channel);
};

class ResonantScanner
{
	FPGAapi mFpga;
	const int mDelayTime = 10;
	double ResonantScanner::convertUm2Volt(double Amplitude);
public:
	bool mState;							//determine if is the scanner on or off
	double mAmplitude_um = 0;
	double mVoltPerUm = RS_voltPerUm;		//Calibration factor. volts per microns
	ResonantScanner(FPGAapi fpga);
	~ResonantScanner();
	NiFpga_Status startStop(bool requestedState);
	NiFpga_Status setOutputVoltage(double Vout);
	NiFpga_Status setOutputAmplitude(double amplitude_um);
	NiFpga_Status turnOn(double amplitude_um);
	NiFpga_Status turnOff();
};

class Shutter
{
	FPGAapi mFpga;
	const int mDelayTime = 10;
public:
	uint32_t mIDshutter;
	bool mState;

	Shutter(FPGAapi fpga, uint32_t ID);
	~Shutter();

	NiFpga_Status setOutput(bool requestedState);
	NiFpga_Status pulseHigh();
};


class Stage
{
public:
	Stage();
	~Stage();
};


class RTsequence
{
	FPGAapi *mFpga;
	const int mLatency_tick = 2;			//latency of detecting the line clock. Calibrate the latency with the oscilloscope
	double ConvertSpatialCoord2Time(double x);
	double getDiscreteTime(int pix);
	double calculateDwellTime(int pix);
	double calculatePracticalDwellTime(int pix);
public:
	RTsequence(FPGAapi *fpga);
	~RTsequence();

	int push(RTchannel chan, QU32 queue);
	int push(RTchannel chan, U32 aa);
	int linearRamp(RTchannel chan, double TimeStep, double RampLength, double Vinitial, double Vfinal);
	QU32 PixelClockEqualDuration();
	QU32 PixelClockEqualDistance();
};



