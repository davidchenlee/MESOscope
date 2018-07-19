#pragma once
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <tiffio.h>					//for Tiff files
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"

class Image
{
	const FPGAapi::Session &mFpga;
	unsigned char *mImage;							//Create a long 1D array containing the image

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

	void startFIFOpc_() const;
	void configureFIFOpc_(const U32 depth) const;			//Currently I don't use this function
	void stopFIFOpc_() const;
	void readRemainingFIFOpc_() const;
	void readFIFOpc_();
	void unpackBuffer_();
	void correctInterleaved_();
	void analyze_() const;
public:
	Image(const FPGAapi::Session &fpga);
	~Image();
	void acquire(const std::string filename = "Untitled");
	void saveAsTiff(std::string filename) const;
	void saveAsTxt(const std::string fileName) const;
};

class Vibratome
{
	const FPGAapi::Session &mFpga;
	const enum VibratomeChannel {VibratomeStart, VibratomeBack, VibratomeForward};		//Vibratome channels
	int mNslide;					//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome(const FPGAapi::Session &fpga);
	~Vibratome();
	void startStop() const;
	void sendCommand(const double dt, const VibratomeChannel channel) const;
};

class ResonantScanner
{
	const FPGAapi::Session &mFpga;
	const int mVMAX_V = 5 * V;										//Max control voltage
	const int mDelay_ms = 10;
	//double mVoltPerUm = 1.285 * V/ ((467 - 264)*um);				//Calibration factor. volts per um. Equal distant pixels, 200 um, 400 pix, 16/July/2018
	double mVoltPerUm = 1.093 * V / (179 * um);						//Calibration factor. volts per um. Equal distant pixels, 170 um, 3400 pix, 16/July/2018
	double mFFOV_um = 0;											//Full field of view
	double mVoltage_V = 0;											//Control voltage 0-5V (max amplitude)
	void setVoltage_(const double Vcontrol_V);
	void setFFOV_(const double FFOV_um);
	double convertUmToVolt_(const double amplitude_um) const;
public:
	ResonantScanner(const FPGAapi::Session &fpga);
	~ResonantScanner();
	void run(const bool state) const;
	void turnOn_um(const double FFOV_um);
	void turnOn_V(const double Vcontrol_V);
	void turnOff();
};

class Shutter
{
	const FPGAapi::Session &mFpga;
	NiFpga_FPGAvi_ControlBool mID;			//Device ID
	const int mDelay_ms = 10;
public:
	Shutter(const FPGAapi::Session &fpga, ShutterID ID);
	~Shutter();
	void open() const;
	void close() const;
	void pulseHigh() const;
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
	int mWavelength_nm;		//Laser wavelength
	double convert_mWToVolt_(const double power_mW) const;
public:
	PockelsCell(FPGAapi::RTsequence &sequence, const RTchannel pockelsChannel, const int wavelength_nm);
	~PockelsCell();
	void pushVoltageSinglet_(const double timeStep, const double AO_V) const;
	void pushPowerSinglet(const double timeStep, const double P_mW) const;
	void voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const;
	void powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const;
	void voltageToZero() const;
	void scalingLinearRamp(const double Si, const double Sf) const;
};

class Galvo
{
	FPGAapi::RTsequence &mSequence;
	RTchannel mGalvoChannel;
	const double voltPerUm = 5.0 * V / (210 * um);		//volts per um. Calibration factor of the galvo. Last calib 11/April/2018
	double convert_umToVolt_(const double position_um) const;
	void pushVoltageSinglet_(const double timeStep, const double AO_V) const;
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
	std::string port = COMport.at(mPMTcom);
	const int mBaud = 9600;
	const int mTimeout_ms = 300;
	const int RxBufferSize = 256;
	uint8_t sumCheck_(const std::vector<uint8_t> input, const int index) const;
	std::vector<uint8_t> sendCommand_(std::vector<uint8_t> command) const;
public:
	mPMT();
	~mPMT();
	void readAllGain() const;
	void setSingleGain(const int channel, const int gain) const;
	void setAllGainToZero() const;
	void setAllGain(const int gain) const;
	void setAllGain(std::vector<uint8_t> gains) const;
	void readTemp() const;
};

class Filterwheel
{
	FilterwheelID mID;			//Device ID
	serial::Serial *mSerial;
	std::string port;
	const int mBaud = 115200;
	const int mTimeout_ms = 150;
	Filtercolor mColor;
	std::string readColorStr_() const;
	void downloadColor_();
public:
	Filterwheel(const FilterwheelID ID);
	~Filterwheel();
	void setColor(const Filtercolor color);
	void setColor(const int wavelength_nm);
};

class Laser
{
	int mWavelength_nm;
	serial::Serial *mSerial;
	const std::string port = COMport.at(VISIONcom);
	const int mBaud = 19200;
	const int mTimeout_ms = 100;
	void downloadWavelength_();
public:
	Laser();
	~Laser();
	void printWavelength_nm() const;
	void setWavelength(const int wavelength_nm);
	void setShutter(const bool state) const;
};

class Stage
{
	const std::string mStageName_x = "116049107";	//X-stage (V-551.4B)
	const std::string mStageName_y = "116049105";	//Y-stage (V-551.2B)
	const std::string mStageName_z = "0165500631";	//Z-stage (ES-100)
	const int mPort_z = 4;							//COMport port
	const int mBaud_z = 38400;
	int3 mID;										//Controller IDs
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mPosition3_mm;							//Absolute position of the stages (x, y, z)
	const double3 mPosMin3_mm{ -60, 0, 10 };			//Min and max positions set by software, which do not necessarily match the values set by hardware (stored in the internal memory of the stages)
	const double3 mPosMax3_mm{ 45, 30, 25 };
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
	void scanningStrategy(const int nTileAbsolute) const;
	double3 readAbsolutePosition3_mm(const int nSection, const int nPlane, const int3 nTileXY) const;
	bool isMoving(const Axis axis) const;
	void waitForMovementToStop(const Axis axis) const;
	void Stage::waitForMovementToStop3() const;
};
