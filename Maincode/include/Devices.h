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
	void moveStage_(const Axis stage, const double position);
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


class Commandline
{
	class MoveStage
	{
	public:
		int sliceNumber;		//Slice number
		int2 stackIJ;			//Indices for the 2D array of stacks
		double2 stackCenter_mm;	//X and Y positiosn of the center of the stack
	};
	class AcqStack
	{
	public:
		int stackNumber;
		int wavelength_nm;
		int scanDirZ;			//Z-stage scan direction: +1 for positive, -1 for negative
		double scanZi_mm;		//Initial z position of a stack-scan
		double stackDepth_um;	//Stack depth or thickness
		double scanPi_mW;		//Initial laser power for a stack-scan
		double stackPinc_mW;	//Laser power increase for a stack-scan
	};
	class CutSlice
	{
	public:
		double3 stagePosition_mm;	//Location of the vibratome blade wrt the stages' origin	
	};
	std::string actionToString_(const Action action) const;
public:
	Action mAction;
	union {
		MoveStage moveStage_;
		AcqStack acqStack_;
		CutSlice cutSlice_;
	} mCommand;

	std::string printHeader() const;
	std::string printHeaderUnits() const;
	void printToFile(std::ofstream *fileHandle) const;
	void printParameters() const;
};

class SampleConfig
{
public:
	ROI mROI_mm;			//Region of interest across the entire sample
	double3 mLength_um;		//Sample size in x, y, and z

	SampleConfig(const ROI roi_mm, const double sampleLengthZ_mm): mROI_mm(roi_mm)
	{
		//Convert input ROI = (xmin, ymax, xmax, ymin) to the equivalent sample length in X and Y
		mLength_um.at(XX) = 1000 * (mROI_mm.at(2) - mROI_mm.at(0));
		mLength_um.at(YY) = 1000 * (mROI_mm.at(1) - mROI_mm.at(3));
		mLength_um.at(ZZ) = 1000 * sampleLengthZ_mm;

		if (mLength_um.at(XX) < 0 || mLength_um.at(YY) < 0 || mLength_um.at(ZZ) < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");
	}
};

class LaserConfig
{
public:
	int mWavelength_nm;		//Laser wavelength
	double mScanPi_mW;		//Initial laser power for a stack-scan 
	double mStackPinc_mW;	//Laser power increase for a stack-scan
};

class StackConfig
{
public:
	double2 mFOV_um;			//Field of view in x and y
	double mStepSizeZ_um;		//Image resolution in z
	double mStackDepth_um;		//Stack depth or thickness
	double3 mStackOverlap_um;	//Stack overlap in x, y, and z

	StackConfig(const double2 FOV_um, const double stepSizeZ_um, const double stackDepth_um, const double3 stackOverlap_um):
		mFOV_um(FOV_um), mStepSizeZ_um(stepSizeZ_um), mStackDepth_um(stackDepth_um), mStackOverlap_um(stackOverlap_um) {}
};

class VibratomeConfig
{
public:
	const double2 mHomePosition_mm{ 0,0 };		//Location of the vibratome blade in x and y wrt the stages origin. Hard-coded parameter
	const double mBladeOffsetZ_um = 35;			//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mSliceThickness_um;					//Slice thickness	

	VibratomeConfig(const double stackDepth_um) : mSliceThickness_um(stackDepth_um){}
};

class Sequencer
{
	const SampleConfig mSample;

	//STACK
	StackConfig mStackConfig;
	int mStackCounter = 0;		//To counter the number of stacks
	int2 mStackArrayDim;		//Dimension of the array of stacks. Value computed dynamically

	//STAGE POSITION
	int3 mScanDir{ 1,1,1 };	//Scan directions in x, y, and z. Initial values all set to 1		
	double mScanZi_mm;		//Initial z-stage position for a stack-scan

	//LASER
	std::vector<LaserConfig> mLaserConfig;

	//VIBRATOME
	VibratomeConfig mVibratomeConfig;
	int mSliceCounter = 0;						//To count the number of the slices
	double mPlaneToCutZ_mm = mScanZi_mm + mVibratomeConfig.mSliceThickness_um / 1000;		//Height of the plane to cut	
	int mNtotalSlices = static_cast<int>(std::ceil(mSample.mLength_um.at(ZZ) / mVibratomeConfig.mSliceThickness_um));	//Number of vibratome slices in the entire sample. Value computed dynamically
	
	double calculateStackScanInitialP_mW_(const double scanPmin_mW, const double stackPinc_mW, const int scanDirZ);
	double2 stackIndicesToStackCenter_mm_(const int2 stackArrayIndices) const;
	void reverseStageScanDirection_(const Axis axis);
	void moveStage_(const int2 stackIJ);
	void acqStack_(const LaserConfig laserConfig);
	void saveStack_();
	void cutSlice_();
public:
	std::vector<Commandline> mCommandList;
	
	Sequencer(const SampleConfig sampleConfig, const std::vector<LaserConfig> laserConfig, const StackConfig stackConfig, const VibratomeConfig vibratomeConfig, const double stageInitialZ_mm);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	void generateCommandlist();
	void printToFile(const std::string fileName) const;
};