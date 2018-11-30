#pragma once
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <algorithm>				//std::max and std::min
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"

class Image
{
	FPGAns::RTsequence &mRTsequence;		//Const because the variables referenced by mRTsequence are not changed by the methods in this class
	U32* mBufArrayA;						//Vector to read FIFOOUTpc A
	U32* mBufArrayB;						//Vector to read FIFOOUTpc B
	TiffU8 mTiff;							//Tiff that store the content of mBufArrayA and mBufArrayB after demultiplexing

	void startFIFOOUTpc_() const;
	void configureFIFOOUTpc_(const U32 depth) const;	//Currently I don't use this function
	void stopFIFOOUTpc_() const;
	void readFIFOOUTpc_();
	void readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, U32* buffer, int &timeout);
	void correctInterleaved_();
	void demultiplex_();
	void FIFOOUTpcGarbageCollector_() const;
public:
	Image(FPGAns::RTsequence &RTsequence);
	~Image();
	//const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence
	void acquire();
	void initialize();
	void startFIFOOUTpc();
	void download();
	void mirrorOddFrames();
	void average();
	void saveTiffSinglePage(std::string filename, const OverrideFileSelector overrideFlag) const;
	void saveTiffMultiPage(std::string filename, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	unsigned char* const accessTiff() const;
};

class ImageException : public std::runtime_error
{
public:
	ImageException(const std::string& message) : std::runtime_error(message.c_str()) {}
};

class Vibratome
{
	const FPGAns::FPGA &mFpga;
	int mNslice;						//Slice number
	double mSectionThickness;			//Thickness of the section
	double mCuttingSpeed_mmps = 0.5;	//in mm/s. Speed of the vibratome for cutting (manual setting)
	double mAmplitude_mm = 1.0;			//in mm. Amplitude of the vibratome for cutting (manual setting)
	double mTravelRange_mm = 52.4;		//in mm. (horizontal) travel range of the head. I measured 104.8 seconds at 0.5 mm/s = 52.4 mm
	double mHeight_mm = 0;				//in mm. vertical distance between the razor blade and the objective's focal plane
	double mMovingSpeed_mmps = 2.495;	//in mm/s. Forward and backward moving speed of the head. 52.4 mm in 21 seconds = 2.495 mm/s

	void moveHead_(const int duration_ms, const VibratomeChannel channel) const;
	void startStop_() const;
public:
	Vibratome(const FPGAns::FPGA &fpga);
	~Vibratome();
	void cutAndRetract(const int distance_mm) const;
	void reset(const int distance_mm) const;
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
	void isRunning();
};

class Galvo
{
	FPGAns::RTsequence &mRTsequence;					//Non-const because some of methods in this class change the variables referenced by mRTsequence	
	RTchannel mGalvoRTchannel;
	const double voltPerUm = 0.02417210 * V / um;		//volts per um. Calibration factor of the galvo. Last calib 31/7/2018
public:
	Galvo(FPGAns::RTsequence &RTsequence, const RTchannel galvoChannel);
	~Galvo();
	//const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence
	void voltageLinearRamp(const double timeStep, const double rampLength, const double Vi_V, const double Vf_V) const;
	void positionLinearRamp(const double timeStep, const double rampLength, const double xi_V, const double xf_V) const;
	void voltageToZero() const;
	void pushVoltageSinglet(const double timeStep, const double AO_V) const;
};

class PMT16X
{
	serial::Serial *mSerial;
	std::string mPort = assignCOM.at(COMPMT16X);
	const int mBaud = 9600;
	const int mTimeout_ms = 300;
	const int mRxBufferSize = 256;				//Serial buffer size

	uint8_t sumCheck_(const std::vector<uint8_t> input, const int index) const;		//The PMT requires a sumcheck. Refer to the manual
	std::vector<uint8_t> sendCommand_(std::vector<uint8_t> command) const;
public:
	PMT16X();
	~PMT16X();
	void readAllGain() const;
	void setSingleGain(const int channel, const int gain) const;
	void setAllGainToZero() const;
	void setAllGain(const int gain) const;
	void setAllGain(std::vector<uint8_t> gains) const;
	void readTemp() const;
};

class Filterwheel
{
	FilterwheelID mDeviceID;				//Device ID = 1, 2, ...
	std::string mDeviceName;				//Device given name
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
	std::string mLaserNameString;
	RTchannel mLaserID;
	int mWavelength_nm;
	serial::Serial *mSerial;
	std::string mPort;
	int mBaud;
	const int mTimeout_ms = 100;
	const double mTuningSpeed_nm_s = 35;				//in nm per second. The measured laser tuning speed is ~ 40 nm/s. Choose a slightly smaller value
	const int mRxBufSize = 256;							//Serial buffer size

	int downloadWavelength_();
public:
	Laser(RTchannel laserID);
	~Laser();
	void printWavelength_nm() const;
	void setWavelength(const int wavelength_nm);
	void setShutter(const bool state) const;
};

class Shutter
{
	const FPGAns::FPGA &mFpga;
	NiFpga_FPGAvi_ControlBool mDeviceID;						//Device ID
	const int mDelay_ms = 10;
public:
	Shutter(const FPGAns::FPGA &fpga, RTchannel laserID);		//I use RTchannel to re-use the laserID. The shutters are not RT
	~Shutter();
	void openClose(const bool state) const;
	void pulseHigh() const;
};

class PockelsCell
{
	FPGAns::RTsequence &mRTsequence;			//Non-const because some methods in this class change the variables referenced by mRTsequence						
	RTchannel mPockelsRTchannel;
	RTchannel mScalingRTchannel;
	int mWavelength_nm;							//Laser wavelength
	const double maxPower_mW = 250 * mW;		//Soft limit for the laser power
	Shutter mShutter;

	double convert_mWToVolt_(const double power_mW) const;
public:
	PockelsCell(FPGAns::RTsequence &RTsequence, const RTchannel laserID, const int wavelength_nm);
	~PockelsCell();
	//const methods do not change the class members. The variables referenced by mRTsequence could change, but not mRTsequence
	void pushVoltageSinglet(const double timeStep, const double AO_V) const;
	void pushPowerSinglet(const double timeStep, const double P_mW, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	void voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const;
	void powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const;
	void voltageToZero() const;
	void scalingLinearRamp(const double Si, const double Sf) const;
	void setShutter(const bool state) const;
};

class VirtualLaser
{
	RTchannel mLaserID;		//Keep track of the current laser being used
	int mWavelength_nm;		//Keep track of the current wavelength being used
	Laser mVision;
	Laser mFidelity;
	PockelsCell mPockelsVision;
	PockelsCell mPockelsFidelity;
	Filterwheel mFWexcitation;
	Filterwheel mFWdetection;
public:
	VirtualLaser(FPGAns::RTsequence &RTsequence, const int wavelength_nm, const double power_mW);
	~VirtualLaser();
	void setWavelength(const int wavelength_nm);
	void pushPowerSinglet(const double timeStep, const double P_mW, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	void setShutter(const bool state) const;
};

class Stage
{
	const int mPort_z = 4;							//COM port
	const int mBaud_z = 38400;
	int3 mID;										//Controller IDs
	const char mNstagesPerController[2] = "1";		//Number of stages per controller (currently 1)
	double3 mPosition3_mm;							//Absolute position of the stages (x, y, z)
	const double3 mPosMin3_mm{ -60, 0, 1 };			//Stage soft limits, which do not necessarily coincide with the values set in hardware (stored in the internal memory of the stages)
	const double3 mPosMax3_mm{ 50, 30, 25 };
	int3 mNtile;									//Tile number in x, y, z
	int3 mNtileOverlap_pix;							//in pixels. Tile overlap in x, y, z

	double qCTO_(const Axis axis, const int chan, const int triggerParam) const;
	bool qTRO_(const Axis axis, const int chan) const;
	void TRO_(const Axis axis, const int chan, const BOOL triggerState) const;

public:
	Stage();
	~Stage();
	double3 readPosition3_mm() const;
	void printPosition3() const;
	void moveStage(const Axis stage, const double position);
	void moveStage3(const double3 positions);
	double downloadPosition_mm(const Axis axis);
	bool isMoving(const Axis axis) const;
	void waitForMotionToStop(const Axis axis) const;
	void waitForMotionToStop3() const;
	void stopALL() const;
	void downloadConfiguration(const Axis axis, const int chan) const;
	double qVEL(const Axis axis) const;
	void VEL(const Axis axis, const double vel_mmPerS) const;


	double3 readAbsolutePosition3_mm(const int nSection, const int nPlane, const int3 nTileXY) const;
};


struct Commandline {
	std::string mAction;		//Image, move stage, cut, etc...
	int mWavelength_nm;
	int3 mStageScanDirection;	//+1 for positive, -1 for negative
	double2 mStackCenter_mm;
	double2 mZiZf_um;
	double2 mPiPf_mW;
	int mSleep_ms;
public:
	Commandline(const std::string action, const int wavelength_nm, const int3 stageScanDirection, const double2 stackCenter_mm, const double2 ZiZf_um, const double2 PiPf_mW, const int sleep_ms) :
		mAction(action), mWavelength_nm(wavelength_nm), mStageScanDirection(stageScanDirection), mStackCenter_mm(stackCenter_mm), mZiZf_um(ZiZf_um), mPiPf_mW(PiPf_mW), mSleep_ms(sleep_ms)
	{};
	~Commandline() {};
	void printCommandline()
	{
		//std::cout << "Action \t\tLambda (nm) \tScan direction \tStack center (mm) \tzi/zf (um) \tPi/Pf (mW) \tSleep (ms)" << std::endl;

		std::cout << mAction << "\t" << mWavelength_nm << "\t\t(" << mStageScanDirection.at(XX) << "," << mStageScanDirection.at(YY) << "," << mStageScanDirection.at(ZZ) << ")\t(";
		std::cout << mStackCenter_mm.at(XX) << "," << mStackCenter_mm.at(YY) << ")\t\t\t" << mZiZf_um.at(0) << "/" << mZiZf_um.at(1) << "\t\t";
		std::cout << mPiPf_mW.at(0) << "/" << mPiPf_mW.at(1) << "\t\t" << mSleep_ms << std::endl;
	}
};

class Sequencer
{
	int nTiles_x = 50;							//Number of tiles in x
	int nTiles_y = 50;							//Number of tiles in y
	int nTilesPerPlane = nTiles_x * nTiles_y;	//Number of tiles in each plane
	int nPlanesPerSlice = 100;					//Number of planes in each slice
	int nSlice = 20;							//Number of slices in the entire sample
	double3 vibratomeHome;
	double3 microscopeHome;
	std::deque <Commandline> queueOfCommandlines;
public:
	Sequencer();
	~Sequencer();
	void scanningStrategy(const int nTileAbsolute) const;
	void pushCommandline(const Commandline commandline);
	void printAllCommandlines();
};