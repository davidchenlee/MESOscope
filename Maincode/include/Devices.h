#pragma once
#include <thread>
#include <fstream>					//file management
#include <ctime>					//Clock()
#include <algorithm>				//std::max and std::min
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"
#include <memory>					//For smart pointers
#include <conio.h>					//For _getch()

class Image
{
	FPGAns::RTcontrol &mRTcontrol;			//Const because the variables referenced by mRTcontrol are not changed by the methods in this class
	U32* mBufArrayA;						//Vector to read FIFOOUTpc A
	U32* mBufArrayB;						//Vector to read FIFOOUTpc B
	TiffU8 mTiff;							//Tiff that store the content of mBufArrayA and mBufArrayB
	ZSCAN mScanDir{ ZSCAN::TOPDOWN };

	void FIFOOUTpcGarbageCollector_() const;
	void readFIFOOUTpc_();
	void readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, U32* buffer, int &timeout);
	void correctInterleaved_();
	void demultiplex_();
	void startFIFOOUTpc_() const;
	void configureFIFOOUTpc_(const U32 depth) const;
	void stopFIFOOUTpc_() const;
public:
	Image(FPGAns::RTcontrol &RTcontrol);
	~Image();
	Image(const Image&) = delete;				//Disable copy-constructor
	Image& operator=(const Image&) = delete;	//Disable assignment-constructor
	Image(Image&&) = delete;					//Disable move constructor
	Image& operator=(Image&&) = delete;			//Disable move-assignment constructor

	void acquire();
	void initialize(const ZSCAN scanDir = ZSCAN::TOPDOWN) const;
	void downloadData();
	void postprocess();
	void averageFrames();
	void averageEvenOddFrames();
	void saveTiffSinglePage(std::string filename, const OVERRIDE override) const;
	void saveTiffMultiPage(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
	U8* const pointerToTiff() const;
};

class ImageException : public std::runtime_error
{
public:
	ImageException(const std::string& message) : std::runtime_error(message.c_str()) {}
};

class ResonantScanner
{
	const FPGAns::RTcontrol &mRTcontrol;					//Needed to retrieve 'mRTcontrol.mWidthPerFrame_pix' to calculate the fill factor
	const double mVMAX{ 5. * V };							//Max control voltage allowed
	const double mDelay{ 10. * ms };
	const double mVoltagePerDistance{ 0.00595 * V / um };	//Calibration factor. Last calibrated 
	double mFullScan;										//Full scan = distance between turning points
	double mControlVoltage;									//Control voltage 0-5V

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
	const double mRampDurationFineTuning{ -250. * us };

	//Scanner
	const double mScanCalib{ 0.02417210 * V / um };			//Calibration factor of the scan galvo. Last calib 31/7/2018

	//Rescanner
	double mRescanVoltageOffset{ 0 };						//Overriden in the constructor because the laser alignment depends on the wavelength

	//For a single laser beam (i.e., without using the beamsplitter) to point at a specific channel of the PMT16X
	const double mInterBeamletDistance = 17.5 * um;			//Set by the beamsplitter specs
	const std::vector<double> beamletOrder{ -7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 0.0 };		//The last entry of the array is for centering the rescanner

	FPGAns::RTcontrol &mRTcontrol;							//Non-const because some methods in this class change the variables referenced by mRTcontrol	
	RTchannel mGalvoRTchannel;
	double mVoltagePerDistance{ 0 };
	int mWavelength_nm{ 0 };
public:
	Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel, const int wavelength_nm = 0);
	Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel, const double posMax, const int wavelength_nm = 0);

	void voltageToZero() const;
	void pushVoltageSinglet(const double timeStep, const double AO) const;
	void voltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf) const;
	void positionLinearRamp(const double timeStep, const double rampLength, const double posInitial, const double posFinal) const;
	void positionLinearRamp(const double posInitial, const double posFinal, const double posOffset = 0) const;
};

class PMT16X
{
	std::unique_ptr<serial::Serial> mSerial;
	COM mPort{ COM::PMT16X };
	const int mBaud{ 9600 };
	const int mTimeout{ 300 * ms };
	const int mRxBufferSize{ 256 };				//Serial buffer size

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
	/* Beamsplitters in the excitation wheel as of Feb 2019
	position #1, 750 nm beamsplitter
	position #2, open (no beamsplitter)
	position #3, 1040 nm beamsplitter
	position #4, open (no beamsplitter)
	position #5, open (no beamsplitter)
	position #6, open (no beamsplitter)
	GREEN and RED are not set up yet. Just doing some tests*/
	const std::vector<FILTERCOLOR> mExcConfig{ FILTERCOLOR::BLUE, FILTERCOLOR::OPEN, FILTERCOLOR::GREEN, FILTERCOLOR::OPEN, FILTERCOLOR::RED, FILTERCOLOR::OPEN };

	/* Filters in the detection wheel as of Feb 2019
	position #1, FF01-492/sp (blue)
	position #2, FF01-520/60 (green)
	position #3, BLP01-532R (red)
	position #4, beam block
	position #5, FF01-514/44 (green)
	position #6, open (no filter)
	Currently, there are 2 green filters set up in the wheel (pos #2 and #5)
	The code looks for the GREEN entry in this vector and reads its position
	For now, I write GREEN in the vector position of the filter to be used
	Place GREEN in the second position to use FF01-520/60, or in the fifth position to use FF01-514/44 (green)*/
	const std::vector<FILTERCOLOR> mDetConfig{ FILTERCOLOR::BLUE, FILTERCOLOR::GREEN, FILTERCOLOR::RED, FILTERCOLOR::CLOSED, FILTERCOLOR::OPEN, FILTERCOLOR::OPEN };

		
	FILTERWHEEL mWhichFilterwheel;	//Device ID = 1, 2, ...
	std::string mFilterwheelName;			//Device given name
	std::vector<FILTERCOLOR> mFWconfig;		//Store the filterwheel configuration for excitation or detection
	FILTERCOLOR mColor;						//Current filterwheel color
	int mPosition;							//Current filterwheel position
	std::unique_ptr<serial::Serial> mSerial;
	COM mPort;
	const int mBaud{ 115200 };
	const int mTimeout{ 150 * ms };
	const int mNpos{ 6 };					//Number of filter positions
	const double mTuningSpeed{ 0.8 / sec };	//The measured filterwheel tuning speed is ~ 1 position/s. Choose a slightly smaller value
	const int mRxBufSize{ 256 };				//Serial buffer size

	void downloadColor_();
	int colorToPosition_(const FILTERCOLOR color) const;
	FILTERCOLOR positionToColor_(const int position) const;
	std::string colorToString_(const FILTERCOLOR color) const;
public:
	Filterwheel(const FILTERWHEEL whichFilterwheel);
	~Filterwheel();
	Filterwheel(const Filterwheel&) = delete;				//Disable copy-constructor
	Filterwheel& operator=(const Filterwheel&) = delete;	//Disable assignment-constructor
	Filterwheel(Filterwheel&&) = delete;					//Disable move constructor
	Filterwheel& operator=(Filterwheel&&) = delete;			//Disable move-assignment constructor

	void setPosition(const FILTERCOLOR color);
	void setWavelength(const int wavelength_nm);
};

class Laser
{
	LASER mWhichLaser;
	int mWavelength_nm;
	std::unique_ptr<serial::Serial> mSerial;
	COM  mPort;
	int mBaud;
	const int mTimeout{ 100 * ms };
	const double mTuningSpeed{ 35. / sec };		//in nm per second. The measured laser tuning speed is ~ 40 nm/s. Choose a slightly smaller value
	const int mRxBufSize{ 256 };				//Serial buffer size

	int downloadWavelength_nm_();
public:
	std::string laserName;
	Laser(const LASER whichLaser);
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
	Shutter(const FPGAns::FPGA &fpga, const LASER whichLaser);
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
	const double timeStep{ 8. * us };
	double mMaxPower;							//Softlimit for the laser power
	Shutter mShutter;

	double laserpowerToVolt_(const double power) const;
public:
	PockelsCell(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LASER laserSelector);	//Do not set the output to 0 through the destructor to allow latching the last value

	void pushVoltageSinglet(const double timeStep, const double AO, const OVERRIDE override = OVERRIDE::DIS) const;
	void pushPowerSinglet(const double timeStep, const double P, const OVERRIDE override = OVERRIDE::DIS) const;
	void voltageToZero() const;
	void voltageLinearRamp(const double Vi, const double Vf) const;
	void powerLinearRamp(const double Pi, const double Pf) const;
	void setShutter(const bool state) const;
	//void voltageLinearRampInFrame(const double timeStep, const double rampDuration, const double Vi, const double Vf) const;
	//void powerLinearRampInFrame(const double timeStep, const double rampDuration, const double Pi, const double Pf) const;
};

class VirtualLaser
{

	LASER mLaserSelect;					//use VISION, FIDELITY, or AUTO (let the code decide)
	LASER mCurrentLaser;				//Laser currently in use: VISION or FIDELITY
	FPGAns::RTcontrol &mRTcontrol;
	int mWavelength_nm;							//Wavelength being used
	Laser mVision;
	Laser mFidelity;
	Filterwheel mFWexcitation;
	Filterwheel mFWdetection;
	std::unique_ptr <PockelsCell> mPockelsPtr;
	const double mPockelTimeStep{ 8. * us };		//Time step for the RT pockels command

	std::string laserNameToString_(const LASER whichLaser) const;
	void isLaserInternalShutterOpen_() const;
	LASER autoselectLaser_(const int wavelength_nm);
	void turnFilterwheels_(const int wavelength_nm);
public:
	VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double initialPower, const double finalPower, const LASER laserSelect = LASER::AUTO);
	VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double laserPower, const LASER laserSelect = LASER::AUTO);
	VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LASER laserSelect = LASER::AUTO);

	void setWavelength(const int wavelength_nm);
	void setPower(const double laserPower) const;
	void setPower(const double initialPower, const double finalPower) const;
	void openShutter() const;
	void closeShutter() const;
};

class Stage
{
	enum StageDOparam { TriggerStep = 1, AxisNumber = 2, TriggerMode = 3, Polarity = 7, StartThreshold = 8, StopThreshold = 9, TriggerPosition = 10 };
	enum StageDOtriggerMode { PositionDist = 0, OnTarget = 2, InMotion = 6, PositionOffset = 7 };

	const int mPort_z{ 4 };							//COM port
	const int mBaud_z{ 38400 };
	int3 mID_XYZ;									//Controller IDs
	const char mNstagesPerController[2]{ "1" };		//Number of stages per controller (currently 1)
	double3 mPositionXYZ;							//Absolute position of the stages
	double3 mVelXYZ;								//Velocity of the stages

	double downloadPositionSingle_(const Axis axis);
	double downloadVelSingle_(const Axis axis) const;
	double downloadDOtriggerParamSingle_(const Axis axis, const int DOchan, const StageDOparam paramId) const;
	void configDOtriggers_() const;
	std::string axisToString(const Axis axis) const;
public:
	const std::vector<double2> mTravelRangeXYZ{ { -65. * mm, 65. * mm }, { -30. * mm, 30. * mm }, { 0. * mm, 26. * mm } };	//Position range of the stages set by hardware. Can not be changed
	const std::vector<double2> mSoftPosLimXYZ{ { -65. * mm, 65. * mm}, { -10. * mm, 30. * mm}, { 1. * mm, 24. * mm} };		//Stage soft limits, which do not necessarily coincide with the values set in hardware (stored in the internal memory of the stages)
	Stage(const double velX, const double velY, const double velZ);
	~Stage();
	Stage(const Stage&) = delete;				//Disable copy-constructor
	Stage& operator=(const Stage&) = delete;	//Disable assignment-constructor
	Stage(Stage&&) = delete;					//Disable move constructor
	Stage& operator=(Stage&&) = delete;			//Disable move-assignment constructor

	double3 readPositionXYZ() const;
	void printPositionXYZ() const;
	void moveSingle(const Axis stage, const double position);
	void moveXY(const double2 positionXY);
	void moveXYZ(const double3 positionXYZ);
	bool isMoving(const Axis axis) const;
	void waitForMotionToStopSingle(const Axis axis) const;
	void waitForMotionToStopAll() const;
	void stopAll() const;
	void setVelSingle(const Axis axis, const double vel);
	void setVelXYZ(const double3 vel);
	void printVelXYZ() const;
	void setDOtriggerParamSingle(const Axis axis, const int DOchan, const StageDOparam paramId, const double value) const;
	void setDOtriggerParamAll(const Axis axis, const int DOchan, const double triggerStep, const StageDOtriggerMode triggerMode, const double startThreshold, const double stopThreshold) const;
	bool isDOtriggerEnabled(const Axis axis, const int DOchan) const;
	void setDOtriggerEnabled(const Axis axis, const int DOchan, const BOOL triggerState) const;
	void printStageConfig(const Axis axis, const int DOchan) const;
};

class Vibratome
{
	const FPGAns::FPGA &mFpga;
	Stage &mStage;

	const double mSlicingVel{ 0.5 * mmps };											//Move the y stage at this velocity for slicing
	const double3 mStageConveyingVelXYZ{ 10. * mmps, 10.  *mmps, 0.5 * mmps };		//Transport the sample between the objective and vibratome at this velocity
	//enum MotionDir { BACKWARD = -1, FORWARD = 1 };
	//double mCuttingSpeed{ 0.5 * mmps };		//Speed of the vibratome for cutting (manual setting)
	//double mMovingSpeed{ 2.495 * mmps };	//Measured moving speed of the head: 52.4 mm in 21 seconds = 2.495 mm/s. Set by hardware. Cannot be changed
	//double mTravelRange{ 52.4 * mm };		//(horizontal) travel range of the head. I measured 104.8 seconds at 0.5 mm/s = 52.4 mm
	//void moveHead_(const double duration, const MotionDir motionDir) const;
	//void cutAndRetractDistance(const double distance) const;
	//void retractDistance(const double distance) const;
public:
	const double2 mStageInitialSlicePosXY{ -53. * mm, 8. * mm };					//Position the stages in front oh the vibratome's blade
	const double mStageFinalSlicePosY{ 27. * mm };									//Final position of the y stage after slicing
	Vibratome(const FPGAns::FPGA &fpga, Stage &stage);
	void pushStartStopButton() const;
	void slice(const double planeToCutZ);
};

class Sample
{
public:
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;
	ROI mROI{ 0, 0, 0, 0 };				//Region of interest across the entire sample {ymin, xmin, ymax, xmax}
	double3 mLengthXYZ{ 0, 0, 0 };		//Sample size in x, y, and z
	double mSurfaceZ{ -1. * mm };

	const double2 mBladePositionXY{ 0. * mm, 0. * mm };	//Location of the vibratome blade in x and y wrt the stages origin
	const double mBladeFocalplaneOffsetZ{ 0. * um };		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack;

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, ROI roi, const double sampleLengthZ, const double sampleSurfaceZ, const double sliceOffset);
	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar);	//For debugging with beads
	void printParams(std::ofstream *fileHandle) const;
};

class Stack
{
public:
	double2 mFFOV;				//Full field of view in x and y
	double mStepSizeZ;			//Image resolution in z
	double mDepth;				//Stack depth or thickness
	double3 mOverlapXYZ_frac;	//Stack overlap in x, y, and z

	Stack(const double2 FFOV, const double stepSizeZ, const int nFrames, const double3 stackOverlapXYZ_frac);
	void printParams(std::ofstream *fileHandle) const;
};

//Create a list of channels
class ChannelList
{
public:
	struct SingleChannel			//Parameters for a single channel
	{
		std::string mName{ "" };	//Channel name
		int mWavelength_nm;			//Laser wavelength
		double mScanPi;				//Initial laser power for a stack-scan. It could be >= or <= than the final laser power depending on the scan direction
		double mStackPinc;			//Laser power increase per unit of distance in Z
	};

	std::vector<SingleChannel> mList;

	ChannelList(const std::vector<SingleChannel> channelList);
	std::size_t size() const;
	SingleChannel front() const;
	SingleChannel at(const int index) const;
	void printParams(std::ofstream *fileHandle) const;
	SingleChannel findChannel(const std::string channel) const;
};

