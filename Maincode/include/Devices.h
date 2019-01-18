#pragma once
#include <thread>
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <algorithm>				//std::max and std::min
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"
#include <memory>					//For smart pointers

class Image
{
	FPGAns::RTcontrol &mRTcontrol;			//Const because the variables referenced by mRTcontrol are not changed by the methods in this class
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
	Image(FPGAns::RTcontrol &RTcontrol);
	~Image();
	Image(const Image&) = delete;				//Disable copy-constructor
	Image& operator=(const Image&) = delete;	//Disable assignment-constructor
	Image(Image&&) = delete;					//Disable move constructor
	Image& operator=(Image&&) = delete;			//Disable move-assignment constructor

	//const methods do not change the class members. The variables referenced by mRTcontrol can be modifiede, but not mRTcontrol itself
	void acquire();
	void initialize();
	void startFIFOOUTpc();
	void download();
	void mirrorOddFrames();
	void averageFrames();
	void averageEvenOddFrames();
	void saveTiffSinglePage(std::string filename, const OverrideFileSelector overrideFlag, const int stackScanDir = 1) const;
	void saveTiffMultiPage(std::string filename, const OverrideFileSelector overrideFlag = NOOVERRIDE, const int stackScanDir = 1) const;
	unsigned char* const pointerToTiff() const;
};

class ImageException : public std::runtime_error
{
public:
	ImageException(const std::string& message) : std::runtime_error(message.c_str()) {}
};

class Vibratome
{
	enum MotionDir { BACKWARD = -1, FORWARD = 1 };
	const FPGAns::FPGA &mFpga;
	double mCuttingSpeed = 0.5 * mmps;	//Speed of the vibratome for cutting (manual setting)
	double mMovingSpeed = 2.495 * mmps;	//Forward and backward moving speed of the head. 52.4 mm in 21 seconds = 2.495 mm/s
	//double mTravelRange = 52.4 * mm;	//(horizontal) travel range of the head. I measured 104.8 seconds at 0.5 mm/s = 52.4 mm

	void moveHead_(const double duration, const MotionDir motionDir) const;
	void pushStartStopButton() const;
public:
	Vibratome(const FPGAns::FPGA &fpga, const double sliceOffset = 0);
	void cutAndRetractDistance(const double distance) const;
	void retractDistance(const double distance) const;
};

class ResonantScanner
{
	const FPGAns::RTcontrol &mRTcontrol;				//Needed to retrieve 'mRTcontrol.mWidthPerFrame_pix' to calculate the fill factor
	const double mVMAX = 5 * V;							//Max control voltage allowed
	const double mDelay = 10 * ms;
	const double mVoltagePerDistance = 0.00595 * V/um;	//Calibration factor. Last calibrated 
	double mFullScan;									//Full scan = distance between turning points
	double mControlVoltage;								//Control voltage 0-5V

	void setVoltage_(const double controlVoltage);
public:
	double mFillFactor;									//Fill factor: how much of an RS swing is covered by the pixels
	double mFFOV;										//Current FFOV
	double mSampRes;									//Spatial sampling resolution (length/pixel)

	ResonantScanner(const FPGAns::RTcontrol &RTcontrol);
	void setFFOV(const double FFOV);
	void turnOn(const double FFOV);
	void turnOnUsingVoltage(const double controlVoltage);
	void turnOff();
	double downloadControlVoltage() const;
	void isRunning() const;
};

class Galvo
{
	FPGAns::RTcontrol &mRTcontrol;								//Non-const because some of methods in this class change the variables referenced by mRTcontrol	
	RTchannel mGalvoRTchannel;
	const double mVoltagePerDistance = 0.02417210 * V/um;		//volts per um. Calibration factor of the galvo. Last calib 31/7/2018
public:
	Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel);
	//const methods do not change the class members. The variables referenced by mRTcontrol could change, but not mRTcontrol
	void voltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf) const;
	void positionLinearRamp(const double timeStep, const double rampLength, const double xi, const double xf) const;
	void generateFrameScan(const double xi, const double xf) const;
	void voltageToZero() const;
	void pushVoltageSinglet(const double timeStep, const double AO) const;
};

class PMT16X
{
	std::unique_ptr<serial::Serial> mSerial;
	int mPort = COMPMT16X;
	const int mBaud = 9600;
	const int mTimeout = 300 * ms;
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
	const std::vector<Filtercolor> mExcConfig{ BLUE, NONE, GREEN, NONE, RED, NONE };
	const std::vector<Filtercolor> mDetConfig{ BLUE, GREEN, RED, NONE, NONE, NONE };
		
	FilterwheelSelector mWhichFilterwheel;	//Device ID = 1, 2, ...
	std::string mFilterwheelName;			//Device given name
	std::vector<Filtercolor> mFWconfig;		//Store the filterwheel configuration for excitation or detection
	Filtercolor mColor;						//Current filterwheel color
	int mPosition;							//Current filterwheel position
	std::unique_ptr<serial::Serial> mSerial;
	int mPort;
	const int mBaud = 115200;
	const int mTimeout = 150 * ms;
	const int mNpos = 6;					//Number of filter positions
	const double mTuningSpeed = 0.8/sec;	//The measured filterwheel tuning speed is ~ 1 position/s. Choose a slightly smaller value
	const int mRxBufSize = 256;				//Serial buffer size

	void downloadColor_();
	void setPosition_(const int position);
	int colorToPosition_(const Filtercolor color) const;
	Filtercolor positionToColor_(const int position) const;
	std::string colorToString_(const Filtercolor color) const;
public:
	Filterwheel(const FilterwheelSelector whichFilterwheel);
	~Filterwheel();
	Filterwheel(const Filterwheel&) = delete;				//Disable copy-constructor
	Filterwheel& operator=(const Filterwheel&) = delete;	//Disable assignment-constructor
	Filterwheel(Filterwheel&&) = delete;					//Disable move constructor
	Filterwheel& operator=(Filterwheel&&) = delete;			//Disable move-assignment constructor
	void setWavelength(const int wavelength_nm);
};

class Laser
{
	LaserSelector mWhichLaser;
	int mWavelength_nm;
	std::unique_ptr<serial::Serial> mSerial;
	int mPort;
	int mBaud;
	const int mTimeout = 100 * ms;
	const double mTuningSpeed = 35./sec;			//in nm per second. The measured laser tuning speed is ~ 40 nm/s. Choose a slightly smaller value
	const int mRxBufSize = 256;					//Serial buffer size

	int downloadWavelength_nm_();
public:
	std::string laserName;
	Laser(const LaserSelector whichLaser);
	~Laser();
	Laser(const Laser&) = delete;				//Disable copy-constructor
	Laser& operator=(const Laser&) = delete;	//Disable assignment-constructor
	Laser(Laser&&) = delete;					//Disable move constructor
	Laser& operator=(Laser&&) = delete;			//Disable move-assignment constructor

	void printWavelength_nm() const;
	void setWavelength(const int wavelength_nm);
	void setShutter(const bool state) const;
	bool isShutterOpen() const;
};

class Shutter
{
	const FPGAns::FPGA &mFpga;
	NiFpga_FPGAvi_ControlBool mWhichShutter;	//Device ID
public:
	Shutter(const FPGAns::FPGA &fpga, const LaserSelector whichLaser);
	~Shutter();
	void setShutter(const bool state) const;
	void pulse(const double pulsewidth) const;
};

class PockelsCell
{
	FPGAns::RTcontrol &mRTcontrol;				//Non-const because some methods in this class change the variables referenced by mRTcontrol						
	RTchannel mPockelsRTchannel;
	RTchannel mScalingRTchannel;
	int mWavelength_nm;							//Laser wavelength
	const double timeStep = 8 * us;
	const double maxPower = 250 * mW;			//Soft limit for the laser power
	Shutter mShutter;

	double laserpowerToVolt_(const double power) const;
	void scalingFactorLinearRamp_(const double Si, const double Sf) const;
public:
	//Do not set the output to 0 through the destructor to allow latching the last value
	PockelsCell(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LaserSelector laserSelector);

	//const methods do not change the class members. The variables referenced by mRTcontrol could change, but not mRTcontrol itself
	void pushVoltageSinglet(const double timeStep, const double AO, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	void pushPowerSinglet(const double timeStep, const double P, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	void voltageToZero() const;
	void voltageLinearRamp(const double Vi, const double Vf) const;
	void powerLinearRamp(const double Pi, const double Pf) const;
	void setShutter(const bool state) const;
	//void voltageLinearRampInFrame(const double timeStep, const double rampDuration, const double Vi, const double Vf) const;
	//void powerLinearRampInFrame(const double timeStep, const double rampDuration, const double Pi, const double Pf) const;
};

class VirtualLaser
{
	const Multiplexing mMultiplexing = SINGLEBEAM;
	FPGAns::RTcontrol &mRTcontrol;
	LaserSelector mLaserSelect;					//VISION, FIDELITY, or AUTO (autoselection)
	LaserSelector mWhichLaser;					//Laser currently used: VISION or FIDELITY
	int mWavelength_nm;							//Wavelength being used
	std::unique_ptr<Laser> mLaserPtr;
	std::unique_ptr <PockelsCell> mPockelsPtr;
	Filterwheel mFWexcitation;
	Filterwheel mFWdetection;
	const double mPockelTimeStep = 8 * us;		//Time step for the RT pockels command

	std::string laserNameToString_(const LaserSelector whichLaser) const;
	void checkShutterIsOpen_(const Laser &laser) const;
	LaserSelector autoSelectLaser_(const int wavelength_nm);
	void tuneFilterwheels_(const int wavelength_nm);
public:
	VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double initialPower = 0, const double powerIncrease = 0, const LaserSelector laserSelect = AUTO);
	void setWavelength(const int wavelength_nm);
	void setPower(const double initialPower, const double powerIncrease = 0) const;
	void openShutter() const;
	void closeShutter() const;
};

class Stage
{
	enum StageDOparam { TriggerStep = 1, AxisNumber = 2, TriggerMode = 3, Polarity = 7, StartThreshold = 8, StopThreshold = 9, TriggerPosition = 10 };
	enum StageDOtriggerMode { PositionDist = 0, OnTarget = 2, InMotion = 6, PositionOffset = 7 };

	const int mPort_z = 4;											//COM port
	const int mBaud_z = 38400;
	int3 mID;														//Controller IDs
	const char mNstagesPerController[2] = "1";						//Number of stages per controller (currently 1)
	double3 mPositionXYZ;											//Absolute position of the stages (x, y, z)
	const double3 mSoftPosMinXYZ{ -60. * mm, 0. * mm, 1. * mm };	//Stage soft limits, which do not necessarily coincide with the values set in hardware (stored in the internal memory of the stages)
	const double3 mSoftPosMaxXYZ{ 50. * mm, 30. * mm, 25. * mm };
	const std::vector<double2> mStagePosLimitXYZ{ {-65. * mm, 65. * mm }, { -30. * mm, 30. * mm }, { 0. * mm, 26. * mm } };	//Position range of the stages
	int3 mNtile;													//Tile number in x, y, z
	int3 mNtileOverlap_pix;											//Tile overlap in x, y, z	

	void configVelAndDOtriggers_(const double3 velXYZ) const;
public:
	Stage(const double velX = 5. * mmps, const double velY = 5. * mmps, const double velZ = 0.02 * mmps);
	~Stage();
	Stage(const Stage&) = delete;				//Disable copy-constructor
	Stage& operator=(const Stage&) = delete;	//Disable assignment-constructor
	Stage(Stage&&) = delete;					//Disable move constructor
	Stage& operator=(Stage&&) = delete;			//Disable move-assignment constructor

	double3 readPositionXYZ() const;
	void printPositionXYZ() const;
	void moveSingleStage(const Axis stage, const double position);
	void moveXYstages(const double2 positionXY);
	void moveXYZstages(const double3 positionXYZ);
	double downloadPosition(const Axis axis);
	bool isMoving(const Axis axis) const;
	void waitForMotionToStopSingleStage(const Axis axis) const;
	void waitForMotionToStopAllStages() const;
	void stopAllstages() const;
	double downloadSingleVelocity(const Axis axis) const;
	void setSingleVelocity(const Axis axis, const double vel) const;
	void setAllVelocities(const double3 vel) const;
	void setDOtriggerSingleParam(const Axis axis, const int DOchan, const StageDOparam paramId, const double value) const;
	void setDOtriggerAllParams(const Axis axis, const int DOchan, const double triggerStep, const StageDOtriggerMode triggerMode, const double startThreshold, const double stopThreshold) const;
	double downloadDOtriggerSingleParam(const Axis axis, const int DOchan, const StageDOparam paramId) const;
	bool isDOtriggerEnabled(const Axis axis, const int DOchan) const;
	void setDOtriggerEnabled(const Axis axis, const int DOchan, const BOOL triggerState) const;
	void printStageConfig(const Axis axis, const int DOchan) const;
};

class Sample
{
public:
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;
	ROI mROI;						//Region of interest across the entire sample
	double3 mLength;				//Sample size in x, y, and z
	const double mInitialZ;

	const double2 mBladePosition{ 0. * mm, 0. * mm };	//Location of the vibratome blade in x and y wrt the stages origin
	const double mBladeFocalplaneOffsetZ = 0 * um;		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack;

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, ROI roi, const double sampleLengthZ, const double initialZ, const double sliceOffset);
	void printParams(std::ofstream *fileHandle) const;
};

class Stack
{
public:
	double2 mFOV;				//Field of view in x and y
	double mStepSizeZ;			//Image resolution in z
	double mDepth;				//Stack depth or thickness
	double3 mOverlap_frac;		//Stack overlap in x, y, and z

	Stack(const double2 FOV, const double stepSizeZ, const double stackDepth, const double3 stackOverlap_frac);
	void printParams(std::ofstream *fileHandle) const;
};

//Create a list of single laser parameters
class LaserList
{
public:
	//Parameters for a single laser
	struct SingleLaser
	{
		int mWavelength_nm;	//Laser wavelength
		double mScanPi;		//Initial laser power for a stack-scan. It could be >= or <= than the final laser power depending on the scan direction
		double mStackPinc;	//Laser power increase for a stack-scan
	};

	std::vector <SingleLaser> mLaser;

	LaserList(const std::vector <SingleLaser> laser);
	std::size_t listSize() const;
	void printParams(std::ofstream *fileHandle) const;
};

