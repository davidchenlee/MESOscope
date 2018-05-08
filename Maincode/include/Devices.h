#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
//#include "UARTscope.h"
#include "Tiffscope.h"
#include <windows.h>	//the stages use this lib. also Sleep
#include <fstream>      //file management
#include <ctime>		//Clock()
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"
using namespace GenericFPGAfunctions;


//Image handling
unsigned char *unpackFIFObuffer(const int counterBufArray_B, int *nElemBufArray_B, U32 **bufArray_B);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrametoTxt(unsigned char *imageArray, const std::string fileName);


class Vibratome
{
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
	void sendCommand(const double dt, const VibratomeChannel channel);
};

class ResonantScanner
{
	const FPGAapi &mFpga;
	const int mVMAX_volt = 5 * V;			//Max control voltage mVcontrol_volt
	const int mDelayTime = 10;
	double mVoltPerUm = RS_voltPerUm;		//Calibration factor. volts per microns
	double mFFOV_um = 0;					//Full field of view
	double mVcontrol_volt = 0;				//Control voltage 0-5V (max amplitude)
	void setVcontrol_volt(const double Vcontrol);
	void setFFOV_um(const double FFOV_um);
	double convertUm2Volt(const double Amplitude);
public:
	ResonantScanner(const FPGAapi &fpga);
	~ResonantScanner();
	void run(const bool state);
	void turnOn_um(const double FFOV_um);
	void turnOn_volt(const double Vcontrol_volt);
	void turnOff();
};

class Shutter
{
	const FPGAapi &mFpga;
	NiFpga_FPGAvi_ControlBool mID;			//Device ID
	const int mDelayTime = 10;
public:
	Shutter(const FPGAapi &fpga, ShutterID ID);
	~Shutter();
	void open();
	void close();
	void pulseHigh();
};

class RTsequence
{
	const FPGAapi &mFpga;
	VQU32 mVectorOfQueues;
	void concatenateQueues(QU32& receivingQueue, QU32& givingQueue);
	QU32 generateLinearRamp(double TimeStep, const double RampLength, const double Vinitial, const double Vfinal);

	//FIFO A
	int nElemReadFIFO_A = 0; 							//Total number of elements read from the FIFO
	U32 *dataFIFO_A;									//The buffer size does not necessarily have to be the size of a frame

	//FIFO B
	int counterBufArray_B = 0;							//Number of buffer arrays actually used
	const int nBufArrays = 100;
	int *nElemBufArray_B;								//Each elements in this array indicates the number of elements in each chunch of data
	int nElemReadFIFO_B = 0; 							//Total number of elements read from the FIFO
	U32 **bufArray_B;									//Each row is used to store the data from the ReadFifo. The buffer size could possibly be < nPixAllFrames

	void configureFIFO(const U32 depth);
	void startFIFOs();
	void readFIFO();
	void stopFIFOs();

	class PixelClock
	{
		const int mLatency_tick = 2;					//latency at detecting the line clock. Calibrate the latency with the oscilloscope
		double ConvertSpatialCoord2Time(const double x);
		double getDiscreteTime(const int pix);
		double calculateDwellTime(const int pix);
		double calculatePracticalDwellTime(const int pix);
	public:
		PixelClock();
		~PixelClock();
		QU32 PixelClockEqualDuration();
		QU32 PixelClockEqualDistance();
	};

public:
	RTsequence(const FPGAapi &fpga);
	RTsequence(const RTsequence&) = delete;	//Disable copy-constructor
	RTsequence& operator=(const RTsequence&) = delete;	//Disable assignment-constructor
	~RTsequence();
	void pushQueue(const RTchannel chan, QU32& queue);
	void pushSingleValue(const RTchannel chan, const U32 input);
	void pushLinearRamp(const RTchannel chan, const double TimeStep, const double RampLength, const double Vinitial, const double Vfinal);
	void loadRTsequenceonFPGA();
	void runRTsequence();
};

class PockelsCell
{
	const FPGAapi &mFpga;
	PockelsID mID;													//Device ID
	NiFpga_FPGAvi_ControlI16 mFPGAid;								//Internal ID of the FPGA
	int mWavelength_nm;												//Wavelength of the laser
	double mVoltPermW = 1;											//Calibration factor
	double mV_volt;													//Output voltage to the HV amplifier
	double voltageforMinPower();									//The output laser power depend on the wavelength
	double convertPowertoVoltage_volt(const double power_mW);
public:
	PockelsCell(const FPGAapi &fpga, const PockelsID ID, const int wavelength_nm);
	~PockelsCell();
	void turnOn_volt(const double V_volt);
	void turnOn_mW(const double power_mW);
	void turnOff();
};

class Filterwheel
{
	FilterwheelID mID;											//Device ID
	std::string port;											//internal ID assigned by the OS
	const int mBaud = 115200;
	const int mTimeout_ms = 150;
	serial::Serial *mSerial;
	FilterColor mPosition;
	void readFilterPosition_();
public:
	Filterwheel(const FilterwheelID ID);
	~Filterwheel();
	void setFilterPosition(const FilterColor color);
	FilterColor readFilterPosition();
};


class Laser
{
	int mWavelength;
	const std::string port = "COM1";						//internal ID assigned by the OS
	const int mBaud = 19200;
	const int mTimeout_ms = 100;
	serial::Serial *mSerial;
	void Laser::readWavelength_();
public:
	Laser();
	~Laser();
	int readWavelength();
	void setWavelength();
};


class Stage
{
	const std::string mStageName_x = "116049107";	//X-stage (V-551.4B)
	const std::string mStageName_y = "116049105";	//Y-stage (V-551.2B)
	const std::string mStageName_z = "0165500631";	//Z-stage (ES-100)
	const int mPort_z = 3;							//COM3
	const int mBaud_z = 38400;
	int3 mID;										//Stage ID assigned by the controllers
	const int mNstages = 3;							//Number of stages (currently 3: X, Y, Z)
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mAbsPosition_mm;						//Absolute position of the stages (x, y, z)
	int3 Ntile;										//Tile number in x, y, z
	int3 tileOverlap_pix;							//in pixels. Tile overlap in x, y, z
public:
	Stage();
	~Stage();
	double3 readPosition_mm();
	void printPosition();
	void moveToPosition_mm(const double position);
	void scanningStrategy(const int nTileAbsolute);
	double retrievePositionForSingleStage_mm(const int ID);
	double3 readAbsolutePosition_mm(const int nSection, const int nPlane, const int3 nTileXY);
};

/*
bool ReferenceIfNeeded(int PIdeviceId, char* axis);
void CloseConnectionWithComment(int PIdeviceId, const char* comment);
void ReportError(int PIdeviceId);
bool referenceStageZ();
*/