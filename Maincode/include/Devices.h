#pragma once
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <tiffio.h>					//for Tiff files
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"

class Vibratome
{
	const FPGAapi::Session &mFpga;
	enum VibratomeChannel {VibratomeStart, VibratomeBack, VibratomeForward};		//Vibratome channels
	int mNslide;					//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome(const FPGAapi::Session &fpga);
	~Vibratome();
	void startStop();
	void sendCommand(const double dt, const VibratomeChannel channel) const;
};

class ResonantScanner
{
	const FPGAapi::Session &mFpga;
	const int mVMAX_V = 5 * V;			//Max control voltage mVcontrol_V
	const int mDelayTime_ms = 10;
	double mVoltPerUm = RS_voltPerUm;	//Calibration factor. volts per microns
	double mFFOV_um = 0;				//Full field of view
	double mVcontrol_V = 0;				//Control voltage 0-5V (max amplitude)
	void setControl_V(const double Vcontrol);
	void setFFOV_um(const double FFOV_um);
	double convertUm2Volt(const double Amplitude) const;
public:
	ResonantScanner(const FPGAapi::Session &fpga);
	~ResonantScanner();
	void run(const bool state);
	void turnOn_um(const double FFOV_um);
	void turnOn_V(const double Vcontrol_V);
	void turnOff();
};

class Shutter
{
	const FPGAapi::Session &mFpga;
	NiFpga_FPGAvi_ControlBool mID;			//Device ID
	const int mDelayTime_ms = 10;
public:
	Shutter(const FPGAapi::Session &fpga, ShutterID ID);
	~Shutter();
	void open() const;
	void close() const;
	void pulseHigh() const;
};

class Image
{
	const FPGAapi::Session &mFpga;
	unsigned char *image;							//Create a long 1D array representing the image

	const int mReadFifoWaitingTime_ms = 15;			//Waiting time between each iteration
	const U32 mTimeout_ms = 100;					//FIFOpc timeout
	int mTimeoutCounter_iter = 100;					//Timeout the while-loop if the FIFO data transfer fails	

	//FIFO A
	U32 mNremainFIFO_A = 0;							//Elements remaining in FIFOpc A
	U32 *mBufArray_A;								//Array to read FIFOpc A
	int mNelemReadFIFO_A = 0; 						//Total number of elements read FIFOpc A

	//FIFO B
	U32 mNremainFIFO_B = 0;							//Elements remaining in the FIFOpc B
	const int nBufArrays = 100;						//Number of buffer arrays to use
	int mCounterBufArray_B = 0;						//Number of buffer arrays actually used
	int *mNelemBufArray_B;							//Each elements in this array indicates the number of elements read in each chunch of data
	U32 **mBufArray_B;								//Each row stores a chunck of data from FIFOpc B. The row size could possibly be < nPixAllFrames
	int mNelemReadFIFO_B = 0; 						//Total number of elements read from FIFOpc B

	void startFIFOpc() const;
	void configureFIFOpc(const U32 depth) const;			//Currently I don't use this function
	void stopFIFOpc() const;
	void readFIFOpc();
	void unpackBuffer();
	void correctInterleavedImage();
	void readRemainingFIFOpc() const;
public:
	Image(const FPGAapi::Session &fpga);
	~Image();
	void acquire(const std::string filename = "Untitled");
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
	FPGAapi::RTsequence &mSequence;									
	RTchannel mPockelsChannel;
	RTchannel mScalingChannel;
	int mWavelength_nm;												//Wavelength of the laser
	double convertPowerToVoltage_V(const double power_mW) const;
public:
	PockelsCell(FPGAapi::RTsequence &sequence, const RTchannel pockelsChannel, const int wavelength_nm);
	~PockelsCell();
	void pushVoltageSinglet(const double timeStep, const double AO_V) const;
	void pushPowerSinglet(const double timeStep, const double P_mW) const;
	void voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const;
	void powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const;
	void voltageToZero() const;
	void scalingLinearRamp(const double Si, const double Sf);
};

class Galvo
{
	FPGAapi::RTsequence &mSequence;
	RTchannel mGalvoChannel;
	const double voltPerUm = 5.0 * V / (210 * um);					//volts per um. Calibration factor of the galvo. Last calib 11/April/2018
	double convertPositionToVoltage(const double position_um) const;
	void pushVoltageSinglet(const double timeStep, const double AO_V) const;
public:
	Galvo(FPGAapi::RTsequence &sequence, const RTchannel galvoChannel);
	~Galvo();
	void voltageLinearRamp(const double timeStep, const double rampLength, const double Vi_V, const double Vf_V) const;
	void positionLinearRamp(const double timeStep, const double rampLength, const double xi_V, const double xf_V) const;
	void voltageToZero() const;
};

class mPMT
{
	serial::Serial *mSerial;
	std::string port = "COM1";
	const int mBaud = 9600;
	const int mTimeout_ms = 300;
	const int RxBufferSize = 256;
	uint8_t sumCheck(const std::vector<uint8_t> input, const int index);
	std::vector<uint8_t> sendCommand(std::vector<uint8_t> command);
public:
	mPMT();
	~mPMT();
	void readAllGain();
	void setSingleGain(const int channel, const int gain);
	void setAllGainToZero();
	void setAllGain(const int gain);
	void setAllGain(std::vector<uint8_t> gains);
	void readTemp();
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
	int mWavelength_nm;
	serial::Serial *mSerial;
	const std::string port = "COM1";						//internal ID assigned by the OS
	const int mBaud = 19200;
	const int mTimeout_ms = 100;
	void Laser::downloadWavelength();
public:
	Laser();
	~Laser();
	int readWavelength_nm() const;
	void setWavelength(const int wavelength_nm);
	void Laser::shutter(const bool state);
};

class Stage
{
	const std::string mStageName_x = "116049107";	//X-stage (V-551.4B)
	const std::string mStageName_y = "116049105";	//Y-stage (V-551.2B)
	const std::string mStageName_z = "0165500631";	//Z-stage (ES-100)
	const int mPort_z = 4;							//COM port
	const int mBaud_z = 38400;
	int3 mID;										//Controller IDs
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mPosition3_mm;							//Absolute position of the stages (x, y, z)
	const double3 mPosMin3_mm{ -60, 0, 10 };			//Min and max positions set by software, which do not necessarily match the values set by hardware (stored in the internal memory of the stages)
	const double3 mPosMax3_mm{ 40, 20, 25 };
	int3 mNtile;									//Tile number in x, y, z
	int3 mNtileOverlap_pix;							//in pixels. Tile overlap in x, y, z
public:
	Stage();
	~Stage();
	double3 readPosition3_mm() const;
	void printPosition3() const;
	void moveStage(const Axis stage, const double position);
	void moveStage3(const double3 positions);
	double downloadPosition_mm(const Axis axis);
	void scanningStrategy(const int nTileAbsolute);
	double3 readAbsolutePosition3_mm(const int nSection, const int nPlane, const int3 nTileXY) const;
	bool isMoving(const Axis axis) const;
	void waitForMovementToStop(const Axis axis) const;
	void Stage::waitForMovementToStop3() const;
};
