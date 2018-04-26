#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
#include "UARTscope.h"
#include "Tiffscope.h"
#include "windows.h"	//the stages use this lib. also Sleep
#include <fstream>      //file management
#include <ctime>		//Clock()
using namespace GenericFPGAfunctions;


//Image handling
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrametoTxt(unsigned char *imageArray, std::string fileName);


class PhotonCounter {
	FPGAapi mFpga;
public:
	PhotonCounter(FPGAapi fpga);
	~PhotonCounter();
	void readCount();
	void readFIFO(int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray);
	void configureFIFO(U32 depth);
};


class Vibratome {
	FPGAapi mFpga;
	enum VibratomeChannel {VibratomeStart, VibratomeBack, VibratomeForward};		//Vibratome channels
	int mNslide;						//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome(FPGAapi fpga);
	~Vibratome();
	void startStop();
	void sendCommand(double dt, VibratomeChannel channel);
};

class ResonantScanner {
	FPGAapi mFpga;
	const int mDelayTime = 10;
	double mVoltPerUm = RS_voltPerUm;		//Calibration factor. volts per microns
	double mAmplitude_um = 0;
	double mAmplitude_volt = 0;
	void setOutputVoltage(double Vout);
	void setOutputAmplitude(double amplitude_um);
	double ResonantScanner::convertUm2Volt(double Amplitude);
public:
	ResonantScanner(FPGAapi fpga);
	~ResonantScanner();
	void startStop(bool state);
	void turnOn(double amplitude_um);
	void turnOff();
};

class Shutter {
	FPGAapi mFpga;
	int mID;			//Device ID
	const int mDelayTime = 10;
public:
	Shutter(FPGAapi fpga, int ID);
	~Shutter();
	void setOutput(bool requestedState);
	void pulseHigh();
};


class Stage
{
	FPGAapi mFpga;
	std::vector<double> absPosition;			//Absolute position of the stages (x, y, z)
	std::vector<int> Ntile;						//Tile number in x, y, z
	std::vector<int> tileOverlap_pix;			//in pixels. Tile overlap in x, y, z
public:
	Stage(FPGAapi fpga);
	~Stage();
};


class RTsequence
{
	FPGAapi mFpga;
	VQU32 mVectorOfQueues;
	void concatenateQueues(QU32& receivingQueue, QU32 givingQueue);

	class PixelClock
	{
		const int mLatency_tick = 2;						//latency at detecting the line clock. Calibrate the latency with the oscilloscope
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

public:
	RTsequence(FPGAapi fpga);
	~RTsequence();
	void pushQueue(RTchannel chan, QU32 queue);
	void pushSingleValue(RTchannel chan, U32 input);
	void pushLinearRamp(RTchannel chan, double TimeStep, double RampLength, double Vinitial, double Vfinal);
	void sendtoFPGA();

};

class Laser
{
	FPGAapi mFpga;
	double mWavelength;
public:
	Laser(FPGAapi fpga);
	~Laser();
};

class PockelsCell
{
	FPGAapi mFpga;
	int mID;
	double mVoltPermW = 1;		//Calibration factor
	double mV_volt;			//Output voltage to the HV amplifier
	double mP_mW;			//Output laser power
	void setOutputVoltage(double V_volt);
public:
	PockelsCell(FPGAapi fpga);
	~PockelsCell();
	void turnOn(double P_mW);
	void turnOff();
};


class Filterwheel
{
	int mID;						//Device ID
	int mPosition;
public:
	Filterwheel(int ID);
	~Filterwheel();
};


