#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

struct MoveStage {
	int mSliceNumber;		//Slice number
	int2 mStackIJ;			//Indices for the 2D array of stacks
	double2 mStackCenterXY;	//X and Y positiosn of the center of the stack
};

struct AcqStack {
	int mStackNumber;
	int mWavelength_nm;
	int mScanDirZ;		//Z-stage scan direction: +1 for positive, -1 for negative
	double mScanZi;		//Initial z position of a stack-scan
	double mStackDepth;	//Stack depth or thickness
	double mScanPi;		//Initial laser power for a stack-scan. It could be >= or <= than the final laser power depending on the scan direction
	double mStackPinc;	//Laser power increase for a stack-scan
};

struct CutSlice {
	double3 mBladePositionXY;		//Position the sample facing the vibratome blade
};

//Single commands
class Commandline
{
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

//A list of commands to form a full sequence
class Sequencer
{
	//Parameters that are unchanged throughout the sequence
	Sample mSample;							//Sample
	const Stack mStack;						//Stack
	const LaserList mLaserList;				//Laser
	const int3 mInitialScanDir{ 1, 1, 1 };	//Initial scan directions in x, y, and z
	ROI mROIcovered;

	//Parameters that vary throughout the sequence
	std::vector<Commandline> mCommandList;
	int mStackCounter{ 0 };				//Count the number of stacks
	int mSliceCounter{ 0 };				//Count the number of the slices
	int2 mStackArrayDimIJ;				//Dimension of the array of stacks. Value computed dynamically
	int3 mScanDir{ mInitialScanDir };	//Scan directions in x, y, and z
	double mScanZi;						//Initial z-stage position for a stack-scan
	double mPlaneToSliceZ;				//Height of the plane to cut	
	int mNtotalSlices;					//Number of vibratome slices in the entire sample

	double calculateStackScanInitialPower_(const double scanPmin, const double stackPinc, const int scanDirZ);
	double2 stackIndicesToStackCenter_(const int2 stackArrayIndicesIJ) const;
	void reverseStageScanDirection_(const Axis axis);
	void resetStageScanDirections_();
	void moveStage_(const int2 stackIJ);
	void acqStack_(const int iterWL);
	void saveStack_();
	void cutSlice_();
public:
	int mCommandCounter{ 0 };

	Sequencer(const LaserList laserList, const Sample sample, const Stack stack);
	Sequencer(const LaserList laserList, Sample sample, const Stack stack, const double3 stackCenterXYZ, const int2 stackArrayDimIJ);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	Commandline readCommandline(const int iterCommandline) const;
	void generateCommandList();
	std::vector<double2> generateLocationList();
	void printSequencerParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
};