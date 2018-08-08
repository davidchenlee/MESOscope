#pragma once
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <algorithm>				//std::max and std::min
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"

class Image
{
	const FPGAns::RTsequence &mRTsequence;			//(Const because the variables referenced by mRTsequence are not changed by the methods of this class)

	std::vector<U32> mBufArrayA;					//Vector to read FIFOOUTpc A
	std::vector<U32> mBufArrayB;					//Vector to read FIFOOUTpc B
	std::vector<unsigned char> mImage;				//Demux image

	void startFIFOOUTpc_() const;
	void configureFIFOOUTpc_(const U32 depth) const;	//Currently I don't use this function
	void stopFIFOOUTpc_() const;
	void readFIFOOUTpc_();
	void readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, std::vector<U32> &buffer);
	void correctInterleaved_();
	void demux_();
	void analyze_() const;
public:
	Image(FPGAns::RTsequence &RTsequence);
	~Image();
	//(const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence)
	void acquire(const bool saveFlag = FALSE, const std::string filename = "Untitled", const bool overrideFile = FALSE);
	void saveTiff(std::string filename, const bool overrideFile) const;
	void saveTxt(const std::string fileName) const;
};

class Vibratome
{
	const FPGAns::FPGA &mFpga;
	const enum VibratomeChannel {VibratomeStart, VibratomeBack, VibratomeForward};		//Vibratome channels
	int mNslide;					//Slide number
	double mSectionThickness;		//Thickness of the section
	double mSpeed;					//Speed of the vibratome (manual setting)
	double mAmplitude;				//Amplitude of the vibratome (manual setting)
public:
	Vibratome(const FPGAns::FPGA &fpga);
	~Vibratome();
	void startStop() const;
	void sendCommand(const double dt, const VibratomeChannel channel) const;
};

class ResonantScanner
{
	const FPGAns::RTsequence &mRTsequence;
	const double mVMAX_V = 5 * V;						//Max control voltage allowed
	const int mDelay_ms = 10;
	double mVoltPerUm = 0.00595;						//Calibration factor. Last calibrated 
	double mFullScan_um;								//Full scan = distance between turning points
	double mControl_V;									//Control voltage 0-5V

	void setVoltage_(const double Vcontrol_V);
public:
	ResonantScanner(const FPGAns::RTsequence &RTsequence);
	~ResonantScanner();
	double mFillFactor;									//Fill factor: how much of an RS swing is covered by the pixels
	double mFFOV_um;									//Current FFOV
	double mSampRes_umPerPix;							//Spatial sampling resolution (um per pixel)

	void setFFOV(const double FFOV_um);
	void turnOn_um(const double FFOV_um);
	void turnOn_V(const double Vcontrol_V);
	void turnOff();
	double downloadControl_V();
	double getSamplingResolution_um();
};

class Shutter
{
	const FPGAns::FPGA &mFpga;
	NiFpga_FPGAvi_ControlBool mID;			//Device ID
	const int mDelay_ms = 10;
public:
	Shutter(const FPGAns::FPGA &fpga, ShutterID ID);
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
	FPGAns::RTsequence &mRTsequence;			//Non-const because some methods in this class change the variables referenced by mRTsequence						
	RTchannel mPockelsChannel;
	RTchannel mScalingChannel;
	int mWavelength_nm;							//Laser wavelength
	const double maxPower_mW = 250 * mW;		//Soft limit for the laser power

	double convert_mWToVolt_(const double power_mW) const;
public:
	PockelsCell(FPGAns::RTsequence &RTsequence, const RTchannel pockelsChannel, const int wavelength_nm);
	~PockelsCell();
	//(const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence)
	void pushVoltageSinglet_(const double timeStep, const double AO_V) const;
	void pushPowerSinglet(const double timeStep, const double P_mW) const;
	void voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const;
	void powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const;
	void voltageToZero() const;
	void scalingLinearRamp(const double Si, const double Sf) const;
};

class Galvo
{
	FPGAns::RTsequence &mRTsequence;					//Non-const because some methods in this class change the variables referenced by mRTsequence	
	RTchannel mGalvoChannel;
	const double voltPerUm = 0.02417210 * V / um;		//volts per um. Calibration factor of the galvo. Last calib 31/7/2018

	double convert_umToVolt_(const double position_um) const;

public:
	Galvo(FPGAns::RTsequence &RTsequence, const RTchannel galvoChannel);
	~Galvo();
	//(const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence)
	void voltageLinearRamp(const double timeStep, const double rampLength, const double Vi_V, const double Vf_V) const;
	void positionLinearRamp(const double timeStep, const double rampLength, const double xi_V, const double xf_V) const;
	void voltageToZero() const;
	void pushVoltageSinglet_(const double timeStep, const double AO_V) const;
};

class mPMT
{
	serial::Serial *mSerial;
	std::string mPort = assignCOM.at(mPMTcom);
	const int mBaud = 9600;
	const int mTimeout_ms = 300;
	const int mRxBufferSize = 256;				//Serial buffer size

	uint8_t sumCheck_(const std::vector<uint8_t> input, const int index) const;		//The PMT requires a sumcheck. Refer to the manual
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
	FilterwheelID mID;						//Device ID
	serial::Serial *mSerial;
	std::string mPort;
	const int mBaud = 115200;
	const int mTimeout_ms = 150;
	Filtercolor mColor;
	const int mNpos = 6;					//Nunmber of filter positions
	const double mTuningSpeed_Hz = 0.8;		//in Hz. The measured filterwheel tuning speed is ~ 1 position/s. Choose a slightly smaller value
	const int mWaitToStop_ms = 3000;		//in ms. Wait until the filterwheel stops turning the turret
	const int mRxBufSize = 256;				//Serial buffer size

	std::string convertToString_(const Filtercolor color) const;
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
	const std::string mPort = assignCOM.at(VISIONcom);
	const int mBaud = 19200;
	const int mTimeout_ms = 100;
	const double mTuningSpeed_nm_s = 35;				//in nm per second. The measured laser tuning speed is ~ 40 nm/s. Choose a slightly smaller value
	const int mRxBufSize = 256;							//Serial buffer size

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
	const int mPort_z = 4;							//COM port
	const int mBaud_z = 38400;
	int3 mID;										//Controller IDs
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mPosition3_mm;							//Absolute position of the stages (x, y, z)
	const double3 mPosMin3_mm{ -60, 0, 1 };			//Stage soft limits, which do not necessarily coincide with the values set in hardware (stored in the internal memory of the stages)
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
	bool isMoving(const Axis axis) const;
	void waitForMovementToStop(const Axis axis) const;
	void Stage::waitForMovementToStop3() const;
	void Stage::stopALL() const;
	void scanningStrategy(const int nTileAbsolute) const;
	double3 readAbsolutePosition3_mm(const int nSection, const int nPlane, const int3 nTileXY) const;
};
