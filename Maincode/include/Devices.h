#pragma once
#include <windows.h>				//the stages use this lib. also Sleep
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <tiffio.h>					//for Tiff files
#include <experimental/filesystem>	//standard method in C++14 but not C++11
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"
#include "FPGAapi.h"

using namespace GenericFPGAfunctions;

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
	const int mDelayTime_ms = 10;
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
	void turnOnSoft_volt(const double Vcontrol_volt);
	void turnOff();
};

class Shutter
{
	const FPGAapi &mFpga;
	NiFpga_FPGAvi_ControlBool mID;			//Device ID
	const int mDelayTime_ms = 10;
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

	class Pixelclock
	{
		QU32 pixelclockQ;					//Queue containing the pixel-clock sequence
		const int mLatency_tick = 2;		//Latency at detecting the line clock. Calibrate the latency with the oscilloscope
		double ConvertSpatialCoord2Time_us(const double x);
		double getDiscreteTime_us(const int pix);
		double calculateDwellTime_us(const int pix);
		double calculatePracticalDwellTime_us(const int pix);
		void equalDuration();
		void equalDistance();
	public:
		Pixelclock();
		~Pixelclock();
		QU32 readPixelclock() const;
	};

public:
	RTsequence(const FPGAapi &fpga);
	RTsequence(const RTsequence&) = delete;				//Disable copy-constructor
	RTsequence& operator=(const RTsequence&) = delete;	//Disable assignment-constructor
	~RTsequence();
	void pushQueue(const RTchannel chan, QU32& queue);
	void pushSingleValue(const RTchannel chan, const U32 input);
	void pushSingleDigitalValue(const RTchannel chan, double t, bool DO);
	void pushSingleAnalogValue(const RTchannel chan, double t, double AO);
	void pushLinearRamp(const RTchannel chan, const double TimeStep, const double RampLength, const double Vinitial, const double Vfinal);
	void uploadRT();
};


class Image
{
	const FPGAapi &mFpga;
	unsigned char *image;							//Create a long 1D array representing the image

	const int mReadFifoWaitingTime_ms = 15;			//Waiting time between each iteration
	const U32 mTimeout_ms = 100;					//FIFO timeout
	int mTimeoutCounter_iter = 100;					//Timeout the while-loop if the FIFO data transfer fails	

	//FIFO A
	U32 mNremainFIFO_A = 0;							//Elements remaining in the FIFO
	U32 *mBufArray_A;								//Array to read the FIFO A
	int mNelemReadFIFO_A = 0; 						//Total number of elements read from the FIFO

	//FIFO B
	U32 mNremainFIFO_B = 0;							//Elements remaining in the FIFO
	const int nBufArrays = 100;						//Number of buffer arrays to use
	int mCounterBufArray_B = 0;						//Number of buffer arrays actually used
	int *mNelemBufArray_B;							//Each elements in this array indicates the number of elements read in each chunch of data
	U32 **mBufArray_B;								//Each row stores a chunck of data from the FIFO. The row size could possibly be < nPixAllFrames
	int mNelemReadFIFO_B = 0; 						//Total number of elements read from FIFO B

	void startFIFO();
	void configureFIFO(const U32 depth);			//Currently I don't use this function
	void readFIFO();
	void unpackBuffer();
	void correctInterleavedImage();
	std::string file_exists(const std::string filename);
public:
	Image(const FPGAapi &fpga);
	~Image();
	void acquire(const std::string filename);
	void stopFIFO();
	void saveAsTiff(std::string filename);
	void saveAsTxt(const std::string fileName);
};

class ImageException : public std::runtime_error
{
public:
	ImageException(const std::string& message) : std::runtime_error(message.c_str()) {}
};


class PockelsCell
{
	const FPGAapi &mFpga;
	PockelsID mID;													//Device ID
	NiFpga_FPGAvi_ControlI16 mFPGAvoltageControllerID;				//Internal ID of the FPGA
	NiFpga_FPGAvi_ControlBool mFPGAselectTriggerControllerID;		//Internal ID of the FPGA
	NiFpga_FPGAvi_ControlBool mFPGAmanualOnControllerID;			//Internal ID of the FPGA
	int mWavelength_nm;												//Wavelength of the laser
	double mV_volt;													//Output voltage to the HV amplifier
	double convertPowertoVoltage_volt(const double power_mW);
public:
	PockelsCell(const FPGAapi &fpga, const PockelsID ID, const int wavelength_nm);
	~PockelsCell();
	void setOutput_volt(const double V_volt);
	void setOutput_mW(const double power_mW);
	void setOutputToZero();
	void manualOn(const bool state);
};

class Filterwheel
{
	FilterwheelID mID;											//Device ID
	serial::Serial *mSerial;
	std::string port;											//internal ID assigned by the OS
	const int mBaud = 115200;
	const int mTimeout_ms = 150;
	FilterColor mPosition;
	void readFilterPosition_();
public:
	Filterwheel(const FilterwheelID ID);
	~Filterwheel();
	void setFilterPosition(const FilterColor color);
	FilterColor readFilterPosition() const;
};

class Laser
{
	int mWavelength;
	serial::Serial *mSerial;
	const std::string port = "COM1";						//internal ID assigned by the OS
	const int mBaud = 19200;
	const int mTimeout_ms = 100;
	void Laser::downloadWavelength();
public:
	Laser();
	~Laser();
	int readWavelength_nm() const;
	void setWavelength();
};

class Stage
{
	const std::string mStageName_x = "116049107";	//X-stage (V-551.4B)
	const std::string mStageName_y = "116049105";	//Y-stage (V-551.2B)
	const std::string mStageName_z = "0165500631";	//Z-stage (ES-100)
	const int mPort_z = 3;							//COM3
	const int mBaud_z = 38400;
	int3 mID;										//Controller IDs
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mPosition_mm;							//Absolute position of the stages (x, y, z)
	const double3 mPosMin_mm{ 30, 0, 10 };				//Min and max positions set by software, which do not necessarily match the values set by hardware (stored in the internal memory of the stages)
	const double3 mPosMax_mm{ 40, 15, 25 };
	int3 mNtile;									//Tile number in x, y, z
	int3 mNtileOverlap_pix;							//in pixels. Tile overlap in x, y, z
public:
	Stage();
	~Stage();
	double3 recallPositionXYZ_mm() const;
	void printPositionXYZ() const;
	void moveStage(const Axis stage, const double position);
	void moveStage(const double3 positions);
	double downloadPosition_mm(const Axis axis);
	void scanningStrategy(const int nTileAbsolute);
	double3 readAbsolutePosition_mm(const int nSection, const int nPlane, const int3 nTileXY) const;
	bool isMoving(const Axis axis) const;
	void waitForMovementStop(const Axis axis) const;
};
