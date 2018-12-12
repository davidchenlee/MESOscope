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
	Image(const Image&) = delete;				//Disable copy-constructor
	Image& operator=(const Image&) = delete;	//Disable assignment-constructor
	Image(Image&&) = delete;					//Disable move constructor
	Image& operator=(Image&&) = delete;			//Disable move-assignment constructor

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
	enum MotionDir { BACKWARD, FORWARD };
	const FPGAns::FPGA &mFpga;
	int mNslice;						//Slice number
	double mSectionThickness;			//Thickness of the section
	double mCuttingSpeed_mmps = 0.5;	//in mm/s. Speed of the vibratome for cutting (manual setting)
	double mAmplitude_mm = 1.0;			//Amplitude of the vibratome for cutting (manual setting)
	double mTravelRange_mm = 52.4;		//(horizontal) travel range of the head. I measured 104.8 seconds at 0.5 mm/s = 52.4 mm
	double mHeight_mm = 0;				//vertical distance between the razor blade and the objective's focal plane
	double mMovingSpeed_mmps = 2.495;	//in mm/s. Forward and backward moving speed of the head. 52.4 mm in 21 seconds = 2.495 mm/s

	void moveHead_(const int duration_ms, const MotionDir motionDir) const;
	void startStop_() const;
public:
	Vibratome(const FPGAns::FPGA &fpga);
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
	PMT16X(const PMT16X&) = delete;				//Disable copy-constructor
	PMT16X& operator=(const PMT16X&) = delete;	//Disable assignment-constructor
	PMT16X(PMT16X&&) = delete;					//Disable move constructor
	PMT16X& operator=(PMT16X&&) = delete;		//Disable move-assignment constructor

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
	const double mTuningSpeed_Hz = 0.8;		//The measured filterwheel tuning speed is ~ 1 position/s. Choose a slightly smaller value
	const int mWaitToStop_ms = 3000;		//Wait until the filterwheel stops turning the turret
	const int mRxBufSize = 256;				//Serial buffer size

	std::string convertToString_(const Filtercolor color) const;
	void downloadColor_();
public:
	Filterwheel(const FilterwheelID ID);
	~Filterwheel();
	Filterwheel(const Filterwheel&) = delete;				//Disable copy-constructor
	Filterwheel& operator=(const Filterwheel&) = delete;	//Disable assignment-constructor
	Filterwheel(Filterwheel&&) = delete;					//Disable move constructor
	Filterwheel& operator=(Filterwheel&&) = delete;			//Disable move-assignment constructor

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
	Laser(const Laser&) = delete;				//Disable copy-constructor
	Laser& operator=(const Laser&) = delete;	//Disable assignment-constructor
	Laser(Laser&&) = delete;					//Disable move constructor
	Laser& operator=(Laser&&) = delete;			//Disable move-assignment constructor

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
	//Do not set the output to 0 through the destructor to allow latching the last value
	PockelsCell(FPGAns::RTsequence &RTsequence, const RTchannel laserID, const int wavelength_nm);

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
	int3 mNtileOverlap_pix;							//Tile overlap in x, y, z

	double qCTO_(const Axis axis, const int chan, const int triggerParam) const;
	bool qTRO_(const Axis axis, const int chan) const;
	void TRO_(const Axis axis, const int chan, const BOOL triggerState) const;

public:
	Stage();
	~Stage();
	Stage(const Stage&) = delete;				//Disable copy-constructor
	Stage& operator=(const Stage&) = delete;	//Disable assignment-constructor
	Stage(Stage&&) = delete;					//Disable move constructor
	Stage& operator=(Stage&&) = delete;			//Disable move-assignment constructor

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
};

//Data types for the commandlines
struct MovParam {
	int vibratomeSliceNumber;
	int2 stackIJ;
	double2 stackCenter_mm;
};
struct AcqParam {
	int stackNumber;
	int wavelength_nm;
	int scanDirZ;				//+1 for positive, -1 for negative
	double2 Z_um;				//Min and max z position
	double2 P_mW;				//Min and max laser power
};

struct CommandParam {
	Action action;
	union {
		struct MovParam movParam;
		struct AcqParam acqParam;
	};
};

class Commandline {
	std::string actionToString_(const Action action) const;
public:
	CommandParam mCommandParam;
	Commandline(const int vibratomeSliceNumber, const int2 stackIJ, const double2 stackCenter_mm);								//Move stage
	Commandline(const int stackNumber, const int wavelength_nm, const int scanDirZ, const double2 Z_um, const double2 P_mW);	//Acq stack
	Commandline(const std::string fileName);																					//Save data
	Commandline();																												//Cut slice
	~Commandline() {};
	std::string printHeader() const;
	std::string printHeaderUnits() const;
	void printToFile(std::ofstream *fileHandle) const;
};

class Sequencer
{
	//STACK ACQUISITION PARAMETERS
	ROI mSampleROI_mm;						//Region of interest across the entire sample
	double2 mSampleSizeXY_um;				//Sample size in x and y
	const double mSampleSizeZ_um = 10000;	//Sample size in z
	double2 mFOV_um{ 150, 200 };			//Field of view in x and y
	double2 mStackOverlap_um{ 0,0 };		//stack overlap in x and y
	int2 mNstackArray;						//Number of stacks in x and y in each vibratome slice
	int mNtotalStacksPerVibratomeSlice;		//Total number of stacks in a vibratome slice
	int mNtotalStackEntireSample;			//Total number of stacks in the entire sample

	double2 stackIndicesToStackCenter_mm_(const int2 stackArrayIndices) const;
public:
	std::vector<Commandline> mCommandList;
	std::vector<int> mWavelengthList_nm;	//Wavelengths

	//STACK ACQUISITION PARAMETERS
	const double stepSizeZ_um = 0.5;

	//VIBRATOME PARAMETERS
	const double2 mVibratomeHome{ 0,0 };
	const double2 mMicroscopeHome{ 0,0 };
	const int mVibratomeSliceThickness_um = 100;
	const int mNvibratomeSlices = static_cast<int>(mSampleSizeZ_um / mVibratomeSliceThickness_um);		//Number of vibratome slices in the entire sample

	Sequencer(const ROI roi_mm, const std::vector<int> wavelengthList_nm);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	void generateCommandlist();
	void printToFile(const std::string fileName) const;
};