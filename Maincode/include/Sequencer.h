#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

//Single commands
class Commandline
{
	struct MoveStage {
		int mSliceNumber;		//Slice number
		int2 stackIJ;			//Indices for the 2D array of stacks
		double2 stackCenter_mm;	//X and Y positiosn of the center of the stack
	};
	struct AcqStack {
		int stackNumber;
		int wavelength_nm;
		int scanDirZ;			//Z-stage scan direction: +1 for positive, -1 for negative
		double scanZi_mm;		//Initial z position of a stack-scan
		double stackDepth_um;	//Stack depth or thickness
		int scanPi_mW;			//Initial laser power for a stack-scan
		int stackPinc_mW;		//Laser power increase for a stack-scan
	};
	struct CutSlice {
		double3 samplePosition_mm;		//Position the sample facing the vibratome blade
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
	std::string sampleName;
	std::string immersionMedium;
	std::string objectiveCollar;
	ROI ROI_mm;				//Region of interest across the entire sample
	double3 length_mm;		//Sample size in x, y, and z

	SampleConfig(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, ROI roi_mm, const double sampleLengthZ_mm) :
		sampleName(sampleName), immersionMedium(immersionMedium), objectiveCollar(objectiveCollar), ROI_mm(roi_mm)
	{
		//Convert input ROI = (xmin, ymax, xmax, ymin) to the equivalent sample length in X and Y
		length_mm.at(XX) = ROI_mm.at(2) - ROI_mm.at(0);
		length_mm.at(YY) = ROI_mm.at(1) - ROI_mm.at(3);
		length_mm.at(ZZ) = sampleLengthZ_mm;

		if (length_mm.at(XX) <= 0 || length_mm.at(YY) <= 0 || length_mm.at(ZZ) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

		if (sampleLengthZ_mm <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Z must be positive");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "SAMPLE ************************************************************\n";
		*fileHandle << "Name = " << sampleName << "\n";
		*fileHandle << "Immersion medium = " << immersionMedium << "\n";
		*fileHandle << std::setprecision(3);
		*fileHandle << "ROI (mm) = [" << ROI_mm.at(0) << "," << ROI_mm.at(1) << "," << ROI_mm.at(2) << "," << ROI_mm.at(3) << "]\n";
		*fileHandle << "Length (mm) = (" << length_mm.at(XX) << "," << length_mm.at(YY) << "," << length_mm.at(ZZ) << ")\n";
		*fileHandle << "\n";
	}
};

//Collect single laser parameters to form a list. The body is defined here
struct LaserListConfig
{
	//Parameters for a single laser
	struct SingleLaserConfig
	{
		int wavelength_nm;	//Laser wavelength
		int scanPi_mW;		//Initial laser power for a stack-scan 
		int stackPinc_mW;	//Laser power increase for a stack-scan
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
			*fileHandle << "Wavelength (nm) = " << mLaserConfig.at(iterWL).wavelength_nm <<
				"\tLaser power (mW) = " << mLaserConfig.at(iterWL).scanPi_mW <<
				"\tPower increase (mW) = " << mLaserConfig.at(iterWL).stackPinc_mW << "\n";
		}
		*fileHandle << "\n";
	}
};

//The body is defined here
struct StackConfig
{
	double2 FOV_um;				//Field of view in x and y
	double stepSizeZ_um;		//Image resolution in z
	double stackDepth_um;		//Stack depth or thickness
	double3 stackOverlap_frac;	//Stack overlap in x, y, and z

	StackConfig(const double2 FOV_um, const double stepSizeZ_um, const double stackDepth_um, const double3 stackOverlap_frac) :
		FOV_um(FOV_um), stepSizeZ_um(stepSizeZ_um), stackDepth_um(stackDepth_um), stackOverlap_frac(stackOverlap_frac)
	{
		if (FOV_um.at(XX) <= 0 || FOV_um.at(YY) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

		if (stepSizeZ_um <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

		if (stackDepth_um <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

		if (stackOverlap_frac.at(XX) < 0 || stackOverlap_frac.at(YY) < 0 || stackOverlap_frac.at(ZZ) < 0
			|| stackOverlap_frac.at(XX) > 0.2 || stackOverlap_frac.at(YY) > 0.2 || stackOverlap_frac.at(ZZ) > 0.2)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range 0-0.2%");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "STACK ************************************************************\n";
		*fileHandle << std::setprecision(1);
		*fileHandle << "FOV (um) = (" << FOV_um.at(XX) << "," << FOV_um.at(YY) << ")\n";
		*fileHandle << "Step size Z (um) = " << stepSizeZ_um << "\n";
		*fileHandle << "Stack depth (um) = " << stackDepth_um << "\n";
		*fileHandle << "Stack overlap (frac) = (" << stackOverlap_frac.at(XX) << "," << stackOverlap_frac.at(YY) << "," << stackOverlap_frac.at(ZZ) << ")\n";
		*fileHandle << "Stack overlap (um) = (" << stackOverlap_frac.at(XX) * FOV_um.at(XX) << "," << stackOverlap_frac.at(YY) * FOV_um.at(YY) << "," << stackOverlap_frac.at(ZZ) * stackDepth_um << ")\n";
		*fileHandle << "\n";
	}
};

//The body is defined here
struct VibratomeConfig
{
	const double2 samplePosition_mm{ 0,0 };			//Location of the vibratome blade in x and y wrt the stages origin. Hard-coded parameter
	const double bladeFocalplaneOffsetZ_um = 0;		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double cutAboveBottomOfStack_um;				//Cut this much above the bottom of the stack

	VibratomeConfig(const double sliceOffset_um) :
		cutAboveBottomOfStack_um(sliceOffset_um)
	{
		if (sliceOffset_um < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be positive");
	}

	void printParams(std::ofstream *fileHandle) const
	{
		*fileHandle << "VIBRATOME ************************************************************\n";
		*fileHandle << std::setprecision(4);
		*fileHandle << "Blade position x,y (mm) = (" << samplePosition_mm.at(XX) << "," << samplePosition_mm.at(YY) << ")\n";
		*fileHandle << std::setprecision(1);
		*fileHandle << "Blade-focal plane vertical offset (um) = " << bladeFocalplaneOffsetZ_um << "\n";
		*fileHandle << "Cut above the bottom of the stack (um) = " << cutAboveBottomOfStack_um << "\n";
		*fileHandle << "\n";
	}
};

//The body is defined here
struct StageConfig
{
	const double stageInitialZ_mm;		//Initial height of the stage
	StageConfig(const double stageInitialZ_mm) :
		stageInitialZ_mm(stageInitialZ_mm){}
};

//The body is defined here
struct GalvoConfig
{
	const Galvo &mGalvo;
	double mFFOV_um;				//Full FOV in the slow axis
	double mTimeStep;				//Time steps size of the linear ramp
	double mPosMax_um;				//Max distance from the center
	double mLinearRampDuration;		//Linear ramp duration

	GalvoConfig(const Galvo &galvo, const double FFOV_um, const double timeStep, const double posMax_um, const double linearRampDuration) :
		mGalvo(galvo), mFFOV_um(FFOV_um), mTimeStep(timeStep), mPosMax_um(posMax_um), mLinearRampDuration(linearRampDuration) {}
};

//The body is defined here
struct ResonantScannerConfig
{
	const ResonantScanner &mRScanner;
	double FFOVrs_um;

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
	double mScanZi_mm;					//Initial z-stage position for a stack-scan
	double mPlaneToSliceZ_mm;			//Height of the plane to cut	
	int mNtotalSlices;					//Number of vibratome slices in the entire sample

	int calculateStackScanInitialP_mW_(const int scanPmin_mW, const int stackPinc_mW, const int scanDirZ);
	double2 stackIndicesToStackCenter_mm_(const int2 stackArrayIndices) const;
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