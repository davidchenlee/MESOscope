#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

//Single commands
class Commandline
{
	struct MoveStage {
		int mSliceNumber;		//Slice number
		int2 mStackIJ;			//Indices for the 2D array of stacks
		double2 mStackCenter;	//X and Y positiosn of the center of the stack
	};
	struct AcqStack {
		int mStackNumber;
		int mWavelength_nm;
		int mScanDirZ;		//Z-stage scan direction: +1 for positive, -1 for negative
		double mScanZi;		//Initial z position of a stack-scan
		double mStackDepth;	//Stack depth or thickness
		int mScanPi;		//Initial laser power for a stack-scan
		int mStackPinc;		//Laser power increase for a stack-scan
	};
	struct CutSlice {
		double3 mSamplePosition;		//Position the sample facing the vibratome blade
	};
	std::string actionToString_(const Action action) const;
public:
	Action mAction;
	union {
		MoveStage moveStage;
		AcqStack acqStack;
		CutSlice cutSlice;
	} mCommand;

	std::string printHeader() const;
	std::string printHeaderUnits() const;
	void printToFile(std::ofstream *fileHandle) const;
	void printParameters() const;
};

//The body is defined here
struct SampleConfig
{
	std::string mSampleName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;
	ROI mROI;			//Region of interest across the entire sample
	double3 mLength;	//Sample size in x, y, and z

	SampleConfig(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, ROI roi, const double sampleLengthZ) :
		mSampleName(sampleName), mImmersionMedium(immersionMedium), mObjectiveCollar(objectiveCollar), mROI(roi)
	{
		//Convert input ROI = (xmin, ymax, xmax, ymin) to the equivalent sample length in X and Y
		mLength.at(XX) = mROI.at(2) - mROI.at(0);
		mLength.at(YY) = mROI.at(1) - mROI.at(3);
		mLength.at(ZZ) = sampleLengthZ;

		if (mLength.at(XX) <= 0 || mLength.at(YY) <= 0 || mLength.at(ZZ) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

		if (sampleLengthZ <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Z must be positive");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "SAMPLE ************************************************************\n";
		*fileHandle << "Name = " << mSampleName << "\n";
		*fileHandle << "Immersion medium = " << mImmersionMedium << "\n";
		*fileHandle << std::setprecision(3);
		*fileHandle << "ROI (mm) = [" << mROI.at(0) /mm << "," << mROI.at(1) / mm << "," << mROI.at(2) / mm << "," << mROI.at(3) / mm << "]\n";
		*fileHandle << "Length (mm) = (" << mLength.at(XX) / mm << "," << mLength.at(YY) / mm << "," << mLength.at(ZZ) / mm << ")\n";
		*fileHandle << "\n";
	}
};

//Collect single laser parameters to form a list. The body is defined here
struct LaserListConfig
{
	//Parameters for a single laser
	struct SingleLaserConfig
	{
		int mWavelength_nm;	//Laser wavelength
		int mScanPi;		//Initial laser power for a stack-scan 
		int mStackPinc;		//Laser power increase for a stack-scan
	};

	std::vector <SingleLaserConfig> mLaserConfig;

	LaserListConfig(const std::vector <SingleLaserConfig> laserConfig) : mLaserConfig(laserConfig) {}

	std::size_t listSize() const
	{
		return mLaserConfig.size();
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "LASER ************************************************************\n";

		for (std::vector<int>::size_type iterWL = 0; iterWL != mLaserConfig.size(); iterWL++)
		{
			*fileHandle << "Wavelength (nm) = " << mLaserConfig.at(iterWL).mWavelength_nm <<
				"\tLaser power (mW) = " << mLaserConfig.at(iterWL).mScanPi / mW <<
				"\tPower increase (mW) = " << mLaserConfig.at(iterWL).mStackPinc / mW << "\n";
		}
		*fileHandle << "\n";
	}
};

//The body is defined here
struct StackConfig
{
	double2 mFOV;				//Field of view in x and y
	double mStepSizeZ;			//Image resolution in z
	double mStackDepth;			//Stack depth or thickness
	double3 mStackOverlap_frac;	//Stack overlap in x, y, and z

	StackConfig(const double2 FOV, const double stepSizeZ, const double stackDepth, const double3 stackOverlap_frac) :
		mFOV(FOV), mStepSizeZ(stepSizeZ), mStackDepth(stackDepth), mStackOverlap_frac(stackOverlap_frac)
	{
		if (FOV.at(XX) <= 0 || FOV.at(YY) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

		if (stepSizeZ <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

		if (stackDepth <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

		if (stackOverlap_frac.at(XX) < 0 || stackOverlap_frac.at(YY) < 0 || stackOverlap_frac.at(ZZ) < 0
			|| stackOverlap_frac.at(XX) > 0.2 || stackOverlap_frac.at(YY) > 0.2 || stackOverlap_frac.at(ZZ) > 0.2)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range 0-0.2%");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "STACK ************************************************************\n";
		*fileHandle << std::setprecision(1);
		*fileHandle << "FOV (um) = (" << mFOV.at(XX) / um << "," << mFOV.at(YY) / um << ")\n";
		*fileHandle << "Step size Z (um) = " << mStepSizeZ / um << "\n";
		*fileHandle << "Stack depth (um) = " << mStackDepth / um << "\n";
		*fileHandle << "Stack overlap (frac) = (" << mStackOverlap_frac.at(XX) << "," << mStackOverlap_frac.at(YY) << "," << mStackOverlap_frac.at(ZZ) << ")\n";
		*fileHandle << "Stack overlap (um) = (" << mStackOverlap_frac.at(XX) * mFOV.at(XX) / um << "," << mStackOverlap_frac.at(YY) * mFOV.at(YY) / um << "," << mStackOverlap_frac.at(ZZ) * mStackDepth << ")\n";
		*fileHandle << "\n";
	}
};

//The body is defined here
struct VibratomeConfig
{
	const double2 mSamplePosition{ 0.*mm,0.*mm };		//Location of the vibratome blade in x and y wrt the stages origin. Hard-coded parameter
	const double mBladeFocalplaneOffsetZ = 0*um;		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack;						//Cut this much above the bottom of the stack

	VibratomeConfig(const double sliceOffset) :
		mCutAboveBottomOfStack(sliceOffset)
	{
		if (sliceOffset < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be positive");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "VIBRATOME ************************************************************\n";
		*fileHandle << std::setprecision(4);
		*fileHandle << "Blade position x,y (mm) = (" << mSamplePosition.at(XX) / mm << "," << mSamplePosition.at(YY) / mm << ")\n";
		*fileHandle << std::setprecision(1);
		*fileHandle << "Blade-focal plane vertical offset (um) = " << mBladeFocalplaneOffsetZ / um << "\n";
		*fileHandle << "Cut above the bottom of the stack (um) = " << mCutAboveBottomOfStack / um << "\n";
		*fileHandle << "\n";
	}
};

//The body is defined here
struct StageConfig
{
	const double mStageInitialZ;		//Initial height of the stage
	StageConfig(const double stageInitialZ) :
		mStageInitialZ(stageInitialZ){}
};

//The body is defined here
struct GalvoConfig
{
	const Galvo &mGalvo;
	double mFFOV;					//Full FOV in the slow axis
	double mTimeStep;				//Time steps size of the linear ramp
	double mPosMax;					//Max distance from the center
	double mLinearRampDuration;		//Linear ramp duration

	GalvoConfig(const Galvo &galvo, const double FFOV, const double timeStep, const double posMax, const double linearRampDuration) :
		mGalvo(galvo), mFFOV(FFOV), mTimeStep(timeStep), mPosMax(posMax), mLinearRampDuration(linearRampDuration) {}
};

//The body is defined here
struct ResonantScannerConfig
{
	const ResonantScanner &mRScanner;

	ResonantScannerConfig(const ResonantScanner &RScanner) : mRScanner(RScanner) {}
};

//A list of commands that form a sequence
class Sequencer
{
	//Parameters that are unchanged throughout the sequence
	const SampleConfig mSample;						//Sample
	const StackConfig mStackConfig;					//Stack
	const LaserListConfig mLaserListConfig;			//Laser
	const VibratomeConfig mVibratomeConfig;			//Vibratome
	const StageConfig mStageConfig;					//Stage
	const int3 mInitialScanDir{ 1,1,1 };			//Initial scan directions in x, y, and z

	//Parameters that vary throughout the sequence
	std::vector<Commandline> mCommandList;
	int mCommandCounter = 0;
	int mStackCounter = 0;				//Count the number of stacks
	int mSliceCounter = 0;				//Count the number of the slices
	int2 mStackArrayDim;				//Dimension of the array of stacks. Value computed dynamically
	int3 mScanDir{ mInitialScanDir };	//Scan directions in x, y, and z
	double mScanZi;						//Initial z-stage position for a stack-scan
	double mPlaneToSliceZ;				//Height of the plane to cut	
	int mNtotalSlices;					//Number of vibratome slices in the entire sample

	int calculateStackScanInitialP_(const int scanPmin, const int stackPinc, const int scanDirZ);
	double2 stackIndicesToStackCenter_(const int2 stackArrayIndices) const;
	void reverseStageScanDirection_(const Axis axis);
	void resetStageScanDirections_();
	void moveStage_(const int2 stackIJ);
	void acqStack_(const int iterWL);
	void saveStack_();
	void cutSlice_();
public:
	Sequencer(const SampleConfig sampleConfig, const LaserListConfig laserListConfig, const StackConfig stackConfig, const VibratomeConfig vibratomeConfig, const StageConfig stageConfig);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	Commandline getCommandline(const int iterCommandline) const;
	void generateCommandList();
	void printSequencerParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
};