#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

struct MoveStage {
	int mSliceNumber;		//Slice number
	int2 mStackIJ;			//Indices of the 2D array of stacks
	double2 mStackCenterXY;	//STAGEX and STAGEY positions corresponding to the center of the stack
};

struct AcqStack {
	int mStackNumber;
	int mWavelength_nm;
	int mScanDirZ;			//+1 for z-stage moving up (top-down imaging) or -1 for z-stage moving down (bottom-up imaging)
	double mScanZi;			//Initial z position of a stack-scan
	double mStackDepth;		//Stack depth or thickness
	double mScanPi;			//Initial laser power for a stack-scan. It could be >= or <= than the final laser power depending on the scan direction
	double mStackPinc;		//Laser power increase in the axis STAGEZ per unit of distance

	double scanZf() const { return  mScanZi + mScanDirZ * mStackDepth; };
	double scanPf() const{ return mScanPi + mScanDirZ * mStackDepth * mStackPinc;};
};

struct CutSlice {
	double3 mBladePositionXY;		//Position the sample facing the vibratome blade
};

//Single commands
class Commandline
{
	std::string actionToString_(const ACTION action) const;
public:
	ACTION mAction;
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
	Sample mSample;							//Sample
	const Stack mStack;						//Stack
	const int3 mInitialScanDir{ 1, 1, 1 };	//Initial scan directions wrt the axis STAGEX, STAGEY, and STAGEZ
	ROI mROIeffective;						//Slightly larger than the requested area: mSample.mROI

	//Parameters that vary throughout the sequence
	std::vector<Commandline> mCommandList;
	int mStackCounter{ 0 };					//Count the number of stacks
	int mSliceCounter{ 0 };					//Count the number of the slices
	int2 mStackArrayDimIJ;					//Dimension of the array of stacks
	int3 mScanDir{ mInitialScanDir };		//Scan directions wrt the axis STAGEX, STAGEY, and STAGEZ
	double mScanZi;							//Initial z-stage position for a stack-scan
	double mPlaneToSliceZ{ 0 };				//Height of the plane to cut	
	int mNtotalSlices{ 1 };					//Number of vibratome slices in the entire sample

	std::vector<U8> polyMask;
	void generatePolyMask_(const std::vector<double2> vertices);

	double calculateStackInitialPower_(const double Ptop, const double stackPinc, const int scanDirZ, const double stackDepth);
	double2 stackIndicesToStackCenter_(const int2 stackArrayIndicesIJ) const;
	void reverseStageScanDirection_(const Axis axis);
	void resetStageScanDirections_();
	double3 effectiveSize_() const;
	void moveStage_(const int2 stackIJ);
	void acqStack_(const int iterWL);
	void saveStack_();
	void cutSlice_();
	int mCommandCounter{ 0 };
public:
	Sequencer(const Sample sample, const Stack stack);
	Sequencer(Sample sample, const Stack stack, const double2 stackCenterXY, const int2 stackArrayDimIJ);
	Sequencer(const Sequencer&) = delete;				//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;	//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;					//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;			//Disable move-assignment constructor

	Commandline readCommandline(const int iterCommandline) const;
	void generateCommandList();
	std::vector<double2> generateLocationList();
	int size() const;
	Stack stack() const;
	void printSequencerParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
};