#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
//#include "UARTscope.h"
#include "Tiffscope.h"
#include "windows.h"	//the stages use this lib. also Sleep
#include <fstream>      //file management
#include <ctime>		//Clock()
using namespace GenericFPGAfunctions;


//Image handling
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrametoTxt(unsigned char *imageArray, std::string fileName);


class Vibratome {
	const FPGAapi &mFpga;
	enum VibratomeChannel {VibratomeStart, VibratomeBack, VibratomeForward};		//Vibratome channels
	int mNslide;					//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome(const FPGAapi &fpga);
	~Vibratome();
	void startStop();
	void sendCommand(double dt, VibratomeChannel channel);
};

class ResonantScanner {
	const FPGAapi &mFpga;
	const int mDelayTime = 10;
	double mVoltPerUm = RS_voltPerUm;		//Calibration factor. volts per microns
	double mAmplitude_um = 0;
	double mAmplitude_volt = 0;
	void setOutputVoltage(double Vout);
	void setOutputAmplitude(double amplitude_um);
	double convertUm2Volt(double Amplitude);
public:
	ResonantScanner(const FPGAapi &fpga);
	~ResonantScanner();
	void startStop(bool state);
	void turnOn(double amplitude_um);
	void turnOff();
};

class Shutter {
	const FPGAapi &mFpga;
	int mID;			//Device ID
	const int mDelayTime = 10;
public:
	Shutter(const FPGAapi &fpga, int ID);
	~Shutter();
	void setOutput(bool requestedState);
	void pulseHigh();
};


class Stage {
	std::vector<double> absPosition;			//Absolute position of the stages (x, y, z)
	std::vector<int> Ntile;						//Tile number in x, y, z
	std::vector<int> tileOverlap_pix;			//in pixels. Tile overlap in x, y, z
public:
	Stage();
	~Stage();
	std::vector<double> getPosition();
};

class RTsequence {
	const FPGAapi &mFpga;
	VQU32 mVectorOfQueues;
	void concatenateQueues(QU32& receivingQueue, QU32 givingQueue);
	QU32 generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal);


	//FIFO A
	int nElemReadFIFO_A = 0; 							//Total number of elements read from the FIFO
	U32 *dataFIFO_A;									//The buffer size does not necessarily have to be the size of a frame

	//FIFO B
	int counterBufArray_B = 0;							//Number of buffer arrays actually used
	const int nBufArrays = 100;
	int *nElemBufArray_B;								//Each elements in this array indicates the number of elements in each chunch of data
	int nElemReadFIFO_B = 0; 							//Total number of elements read from the FIFO
	U32 **bufArray_B;									//Each row is used to store the data from the ReadFifo. The buffer size could possibly be < nPixAllFrames

	void configureFIFO(U32 depth);
	void startFIFOs();
	void readFIFO();
	void stopFIFOs();

	class PixelClock
	{
		const int mLatency_tick = 2;					//latency at detecting the line clock. Calibrate the latency with the oscilloscope
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
	RTsequence(const FPGAapi &fpga);
	~RTsequence();
	void pushQueue(RTchannel chan, QU32 queue);
	void pushSingleValue(RTchannel chan, U32 input);
	void pushLinearRamp(RTchannel chan, double TimeStep, double RampLength, double Vinitial, double Vfinal);
	void loadRTsequenceonFPGA();
	void runRTsequence();
};


class Laser {
	double mWavelength;
public:
	Laser();
	~Laser();
};

class PockelsCell{
	const FPGAapi &mFpga;
	PockelsID mID;													//Device ID
	NiFpga_FPGAvi_ControlI16 mFPGAid;								//internal ID assigned by the FPGA
	double mVoltPermW = 1;											//Calibration factor
	double mV_volt;													//Output voltage to the HV amplifier
	double mP_mW;													//Output laser power
	void setOutputVoltage(double V_volt);
public:
	PockelsCell(const FPGAapi &fpga, PockelsID ID);
	~PockelsCell();
	void turnOn(double P_mW);
	void turnOff();
};

class Filterwheel {
	FilterwheelID mID;						//Device ID
	std::string COM = "";					//internal ID assigned by the OS
	int mPosition;
public:
	Filterwheel(FilterwheelID ID);
	~Filterwheel();
};