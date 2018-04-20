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
	int Nslide;						//Slide number
	double sectionThickness;		//Thickness of the section
	double speed;					//Speed of the vibratome (manual setting)
	double amplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome();
	~Vibratome();
	int startStop(NiFpga_Status* status, NiFpga_Session session);
	int sendCommand(NiFpga_Status* status, NiFpga_Session session, double dt, VibratomeChannel channel);
};



//Resonant scanner
NiFpga_Status resonantScanner_StartStop(NiFpga_Status* status, NiFpga_Session session, bool state);
int resonantScanner_SetOutputVoltager(NiFpga_Status* status, NiFpga_Session session, double Vout);
double resonantScanner_Amp2Volt(double Amplitude);

//Shutters
int shutter1_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);
int shutter2_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);

class PixelClock
{
	U32Q Queue;
	const int latency_tick = 2;		//latency of detecting the line clock. Calibrate the latency with the oscilloscope
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

