#pragma once
#include "Utilities.h"
using namespace Constants;

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
		double3 bladePosition_mm;	//Location of the vibratome blade wrt the stages' origin	
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

class SampleConfig
{
public:
	std::string sampleName;
	ROI ROI_mm;				//Region of interest across the entire sample
	double3 length_um;		//Sample size in x, y, and z

	SampleConfig(const std::string sampleName, ROI roi_mm, const double sampleLengthZ_mm) : sampleName(sampleName), ROI_mm(roi_mm)
	{
		//Convert input ROI = (xmin, ymax, xmax, ymin) to the equivalent sample length in X and Y
		length_um.at(XX) = 1000 * (ROI_mm.at(2) - ROI_mm.at(0));
		length_um.at(YY) = 1000 * (ROI_mm.at(1) - ROI_mm.at(3));
		length_um.at(ZZ) = 1000 * sampleLengthZ_mm;

		if (length_um.at(XX) <= 0 || length_um.at(YY) <= 0 || length_um.at(ZZ) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

		if (sampleLengthZ_mm <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Z must be positive");
	}
};

class LaserConfig
{
public:
	int wavelength_nm;		//Laser wavelength
	double scanPi_mW;		//Initial laser power for a stack-scan 
	double stackPinc_mW;	//Laser power increase for a stack-scan
};

class StackConfig
{
public:
	double2 FOV_um;				//Field of view in x and y
	double stepSizeZ_um;		//Image resolution in z
	double stackDepth_um;		//Stack depth or thickness
	double3 stackOverlap_um;	//Stack overlap in x, y, and z

	StackConfig(const double2 FOV_um, const double stepSizeZ_um, const double stackDepth_um, const double3 stackOverlap_um) :
		FOV_um(FOV_um), stepSizeZ_um(stepSizeZ_um), stackDepth_um(stackDepth_um), stackOverlap_um(stackOverlap_um)
	{
		if (FOV_um.at(XX) <= 0 || FOV_um.at(YY) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

		if (stepSizeZ_um <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

		if (stackDepth_um <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

		if (stackOverlap_um.at(XX) <= 0 || stackOverlap_um.at(YY) <= 0 || stackOverlap_um.at(ZZ) <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be positive");
	}
};

class VibratomeConfig
{
public:
	const double2 homePosition_mm{ 0,0 };		//Location of the vibratome blade in x and y wrt the stages origin. Hard-coded parameter
	const double bladeOffsetZ_um = 35;			//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double sliceThickness_um;					//Slice thickness	

	VibratomeConfig(const double stackDepth_um) : sliceThickness_um(stackDepth_um)
	{
		if (sliceThickness_um <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice thickness must be positive");
	}
};

class Sequencer
{
	//Scan parameters (unchanged throughout the sequence)
	const SampleConfig mSample;				//Sample
	const StackConfig mStackConfig;				//Stack
	const std::vector<LaserConfig> mLaserConfig;	//Laser
	const VibratomeConfig mVibratomeConfig;		//Vibratome

	//Sequencer variables (vary throughout the sequence)
	std::vector<Commandline> mCommandList;
	int mStackCounter = 0;		//To counter the number of stacks
	int mSliceCounter = 0;		//To count the number of the slices
	int2 mStackArrayDim;		//Dimension of the array of stacks. Value computed dynamically
	int3 mScanDir{ 1,1,1 };		//Scan directions in x, y, and z. Initial values all set to 1		
	double mScanZi_mm;			//z-stage position for a stack-scan
	double mPlaneToCutZ_mm = mScanZi_mm + mVibratomeConfig.sliceThickness_um / 1000;								//Height of the plane to cut	
	int mNtotalSlices = static_cast<int>(std::ceil(mSample.length_um.at(ZZ) / mVibratomeConfig.sliceThickness_um));	//Number of vibratome slices in the entire sample. Value computed dynamically

	double calculateStackScanInitialP_mW_(const double scanPmin_mW, const double stackPinc_mW, const int scanDirZ);
	double2 stackIndicesToStackCenter_mm_(const int2 stackArrayIndices) const;
	void reverseStageScanDirection_(const Axis axis);
	void moveStage_(const int2 stackIJ);
	void acqStack_(const LaserConfig laserConfig);
	void saveStack_();
	void cutSlice_();
public:
	Sequencer(const SampleConfig sampleConfig, const std::vector<LaserConfig> laserConfig, const StackConfig stackConfig, const VibratomeConfig vibratomeConfig, const double stageInitialZ_mm);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	Commandline getCommandline(const int iterCommandline) const;
	void generateCommandList();
	void printToFile(const std::string fileName) const;
};