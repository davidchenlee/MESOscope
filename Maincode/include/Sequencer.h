#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

struct FluorLabelList	//Create a list of fluorescent labels
{
	struct FluorLabel //Parameters for a single fluorescent label
	{
		std::string mName{ "" };	//Fluorescent label name
		int mWavelength_nm;			//Laser wavelength
		double mScanPi;				//Initial laser power for a stack-scan. It could be >= or <= than the final laser power depending on the scan direction
		double mStackPinc;			//Laser power increase per unit of distance in the axis STAGEZ
	};
	std::vector<FluorLabel> mFluorLabelList;
	FluorLabelList(const std::vector<FluorLabel> fluorLabelList);
	std::size_t size() const;
	FluorLabel front() const;
	FluorLabel at(const int index) const;
	void printParams(std::ofstream *fileHandle) const;
	FluorLabel findFluorLabel(const std::string fluorLabel) const;
};

struct Sample
{
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;						
	ROI4 mROIrequest{ 0, 0, 0, 0 };						//Requested ROI4 across the entire sample { YMIN, XMIN, YMAX, XMAX };
	SAMPLESIZE3 mSizeRequest{ 0, 0, 0 };						//Requested sample size in the axis STAGEX, STAGEY, and STAGEZ
	double mSurfaceZ{ -1. * mm };
	FluorLabelList mFluorLabelList;
	std::vector<LIMIT2> mStageSoftPosLimXYZ;				//Soft position limits of the stages

	const POSITION2 mBladePositionXY{ 0. * mm, 0. * mm };		//Location of the vibratome blade in the axis STAGEX and STAGEY wrt the stages origin
	const double mBladeFocalplaneOffsetZ{ 0. * um };		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack{ 0. * um };				//Specify at what height of the overlapping volume to cut

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList = { {} });
	Sample(const Sample& sample, ROI4 roi, const double sampleLengthZ, const double sampleSurfaceZ, const double sliceOffset);
	FluorLabelList::FluorLabel findFluorLabel(const std::string fluorLabel) const;
	void printParams(std::ofstream *fileHandle) const;
};

struct Stack
{
	FFOV2 mFFOV;				//Full field of view in the axis STAGEX and STAGEY
	double mStepSizeZ;			//Image resolution in the axis STAGEZ
	double mDepth;				//Stack depth or thickness
	TILEOVERLAP4 mOverlap_frac;	//Stack overlap in the axis STAGEX, STAGEY, and STAGEZ

	Stack(const FFOV2 FFOV, const double stepSizeZ, const int nFrames, const TILEOVERLAP4 stackOverlap_frac);
	void printParams(std::ofstream *fileHandle) const;
};

namespace Action
{
	enum class ID { CUT, ACQ, SAV, MOV };
	struct MoveStage {
		int mSliceNumber;		//Slice number
		INDICES2 mStackIJ;			//Indices of the 2D array of stacks
		POSITION2 mStackCenterXY;	//STAGEX and STAGEY positions corresponding to the center of the stack
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
		double scanPf() const { return mScanPi + mScanDirZ * mStackDepth * mStackPinc; };
	};
	struct CutSlice {
		POSITION3 mBladePositionXY;		//Position the sample facing the vibratome blade
	};
}

class Sequence	//A list of commands that forms a full sequence
{
public:
	class Commandline	//Single commands
	{
	public:
		Action::ID mAction;
		union {
			Action::MoveStage moveStage;
			Action::AcqStack acqStack;
			Action::CutSlice cutSlice;
		} mCommand;

		std::string printHeader() const;
		std::string printHeaderUnits() const;
		void printToFile(std::ofstream *fileHandle) const;
		void printParameters() const;
	private:
		std::string actionToString_(const Action::ID action) const;
	};
	Sequence(const Sample sample, const Stack stack);
	Sequence(Sample sample, const Stack stack, const POSITION2 stackCenterXY, const INDICES2 stackArrayDimIJ);
	Sequence(const Sequence&) = delete;				//Disable copy-constructor
	Sequence& operator=(const Sequence&) = delete;	//Disable assignment-constructor
	Sequence(Sequence&&) = delete;					//Disable move constructor
	Sequence& operator=(Sequence&&) = delete;		//Disable move-assignment constructor

	Commandline readCommandline(const int iterCommandline) const;
	void generateCommandList();
	std::vector<POSITION2> generateLocationList();
	int size() const;
	Stack stack() const;
	void printSequenceParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
private:
	Sample mSample;							//Sample
	const Stack mStack;						//Stack
	std::vector<Commandline> mCommandList;
	const int3 mInitialScanDir{ 1, 1, 1 };	//Initial scan directions wrt the axis STAGEX, STAGEY, and STAGEZ					
	ROI4 mROIeff;							//Slightly larger than the requested area: mSample.mROI
	int mStackCounter{ 0 };					//Count the number of stacks
	int mSliceCounter{ 0 };					//Count the number of the slices
	INDICES2 mStackArrayDimIJ;				//Dimension of the array of stacks
	int3 mScanDir{ mInitialScanDir };		//Scan directions wrt the axis STAGEX, STAGEY, and STAGEZ
	double mScanZi;							//Initial z-stage position for a stack-scan
	double mPlaneToSliceZ{ 0 };				//Height of the plane to cut	
	int mNtotalSlices{ 1 };					//Number of vibratome slices in the entire sample

	double calculateStackInitialPower_(const double Ptop, const double stackPinc, const int scanDirZ, const double stackDepth);
	POSITION2 stackIndicesToStackCenter_(const INDICES2 stackArrayIndicesIJ) const;
	void reverseStageScanDirection_(const Stage::Axis axis);
	void resetStageScanDirections_();
	SAMPLESIZE3 effectiveSize_() const;
	void moveStage_(const INDICES2 stackIJ);
	void acqStack_(const int iterWL);
	void saveStack_();
	void cutSlice_();
	int mCommandCounter{ 0 };
};