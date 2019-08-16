#include "Sequencer.h"

#pragma region "FluorLabelList"
FluorLabelList::FluorLabelList(const std::vector<FluorLabel> fluorLabelList) : mFluorLabelList{ fluorLabelList } {}

std::size_t FluorLabelList::size() const
{
	return mFluorLabelList.size();
}

FluorLabelList::FluorLabel FluorLabelList::front() const
{
	return mFluorLabelList.front();
}

FluorLabelList::FluorLabel FluorLabelList::at(const int index) const
{
	return mFluorLabelList.at(index);
}

void FluorLabelList::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "ID ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mFluorLabelList.size(); iterWL++)
	{
		*fileHandle << "Wavelength (nm) = " << mFluorLabelList.at(iterWL).mWavelength_nm <<
			"\nPower (mW) = " << mFluorLabelList.at(iterWL).mScanPi / mW <<
			"\nPower increase (mW/um) = " << mFluorLabelList.at(iterWL).mStackPinc / mWpum << "\n";
	}
	*fileHandle << "\n";
}

//Return the first instance of "fluorLabel" in mFluorLabelList
FluorLabelList::FluorLabel FluorLabelList::findFluorLabel(const std::string fluorLabel) const
{
	for (std::vector<int>::size_type iter_label = 0; iter_label < mFluorLabelList.size(); iter_label++)
	{
		if (!fluorLabel.compare(mFluorLabelList.at(iter_label).mName)) //compare() returns 0 if the strings are identical
			return mFluorLabelList.at(iter_label);
	}
	//If the requested fluorLabel is not found
	throw std::runtime_error((std::string)__FUNCTION__ + ": Fluorescent label " + fluorLabel + " not found");
}
#pragma endregion "FluorLabelList"

#pragma region "Sample"
Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList) :
	mName{ sampleName }, mImmersionMedium{ immersionMedium }, mObjectiveCollar{ objectiveCollar }, mStageSoftPosLimXYZ{ stageSoftPosLimXYZ }, mFluorLabelList{ fluorLabelList }{}

Sample::Sample(const Sample& sample, ROI4 roi, const double sampleLengthZ, const double sampleSurfaceZ, const double sliceOffset) :
	mName{ sample.mName }, mImmersionMedium{ sample.mImmersionMedium }, mObjectiveCollar{ sample.mObjectiveCollar }, mFluorLabelList{ sample.mFluorLabelList }, mROIrequest{ roi }, mSurfaceZ{ sampleSurfaceZ }, mCutAboveBottomOfStack{ sliceOffset }
{
	//Convert input ROI4 = {ymin, xmin, ymax, xmax} to the equivalent sample length in the axis STAGEX and STAGEY
	mSizeRequest.XX = mROIrequest.XMAX - mROIrequest.XMIN;
	mSizeRequest.YY = mROIrequest.YMAX - mROIrequest.YMIN;
	mSizeRequest.ZZ = sampleLengthZ;

	if (mSizeRequest.XX <= 0 || mSizeRequest.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

	if (mSizeRequest.ZZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Z must be positive");

	if (mCutAboveBottomOfStack < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be positive");
}

FluorLabelList::FluorLabel Sample::findFluorLabel(const std::string fluorLabel) const
{
	return mFluorLabelList.findFluorLabel(fluorLabel);
}

void Sample::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SAMPLE ************************************************************\n";
	*fileHandle << "Name = " << mName << "\n";
	*fileHandle << "Immersion medium = " << mImmersionMedium << "\n";
	*fileHandle << "Correction collar = " << mObjectiveCollar << "\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Requested ROI [STAGEYmin, STAGEXmin, STAGEYmax, STAGEXmax] (mm) = [" << mROIrequest.YMIN / mm << ", " << mROIrequest.XMIN / mm << ", " << mROIrequest.YMAX / mm << ", " << mROIrequest.XMAX / mm << "]\n";
	*fileHandle << "Requested sample size (STAGEX, STAGEY, STAGEZ) (mm) = (" << mSizeRequest.XX / mm << ", " << mSizeRequest.YY / mm << ", " << mSizeRequest.ZZ / mm << ")\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Blade position (STAGEX, STAGEY) (mm) = (" << mBladePositionXY.XX / mm << ", " << mBladePositionXY.YY / mm << ")\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset (um) = " << mBladeFocalplaneOffsetZ / um << "\n";
	*fileHandle << "Cut above the bottom of the stack (um) = " << mCutAboveBottomOfStack / um << "\n";
	*fileHandle << "\n";
}
#pragma endregion "Sample"

#pragma region "Stack"
Stack::Stack(const FFOV2 FFOV, const double stepSizeZ, const int nFrames, const TILEOVERLAP4 overlap_frac) :
	mFFOV{ FFOV }, mStepSizeZ{ stepSizeZ }, mDepth{ stepSizeZ *  nFrames }, mOverlap_frac{ overlap_frac }
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

	if (mStepSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

	if (mDepth <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

	if (mOverlap_frac.XX < 0 || mOverlap_frac.YY < 0 || mOverlap_frac.ZZ < 0
		|| mOverlap_frac.XX > 0.2 || mOverlap_frac.YY > 0.2 || mOverlap_frac.ZZ > 0.2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-0.2]");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (STAGEX, STAGEY) (um) = (" << mFFOV.XX / um << ", " << mFFOV.YY / um << ")\n";
	*fileHandle << "Step size Z (um) = " << mStepSizeZ / um << "\n";
	*fileHandle << "Stack depth (um) = " << mDepth / um << "\n";
	*fileHandle << std::setprecision(2);
	*fileHandle << "Stack overlap (frac) = (" << mOverlap_frac.XX << ", " << mOverlap_frac.YY << ", " << mOverlap_frac.ZZ << ")\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Stack overlap (um) = (" << mOverlap_frac.XX * mFFOV.XX / um << ", " << mOverlap_frac.YY * mFFOV.YY / um << ", " << mOverlap_frac.ZZ * mDepth << ")\n";
	*fileHandle << "\n";
}
#pragma endregion "Stack"

#pragma region "Commandline"
std::string Sequence::Commandline::printHeader() const
{
	return 	"Action\tSlice#\tStackIJ\t(STAGEX,STAGEY)\tStack#\tWavlen\tDirZ\tSTAGEZi\tSTAGEZf\tPi\tPf";
}

std::string Sequence::Commandline::printHeaderUnits() const
{
	return "\t\t(mm,mm)\t\t\tnm\t\tmm\tmm\tmW\tmW";
}

void Sequence::Commandline::printToFile(std::ofstream *fileHandle) const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mCommand.moveStage.mSliceNumber;
		*fileHandle << "\t(" << mCommand.moveStage.mStackIJ.II << "," << mCommand.moveStage.mStackIJ.JJ << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mCommand.moveStage.mStackCenterXY.XX / mm << "," << mCommand.moveStage.mStackCenterXY.YY / mm << ")\n";
		break;
	case Action::ID::ACQ:
		*fileHandle << actionToString_(mAction) << "\t\t\t\t\t";
		*fileHandle << mCommand.acqStack.mStackNumber << "\t";
		*fileHandle << mCommand.acqStack.mWavelength_nm << "\t";
		*fileHandle << mCommand.acqStack.mScanDirZ << "\t";
		*fileHandle << std::setprecision(3);
		*fileHandle << mCommand.acqStack.mScanZi / mm << "\t";
		*fileHandle << mCommand.acqStack.scanZf() / mm << "\t";
		*fileHandle << std::setprecision(0);
		*fileHandle << mCommand.acqStack.mScanPi << "\t" << mCommand.acqStack.scanPf() << "\n";
		break;
	case Action::ID::SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case Action::ID::CUT:
		*fileHandle << actionToString_(mAction);
		*fileHandle << std::setprecision(3);
		*fileHandle << "\t************Sample facing the vibratome at (mm) = ";
		*fileHandle << "(" << mCommand.cutSlice.mBladePositionXY.XX / mm << "," << mCommand.cutSlice.mBladePositionXY.YY / mm << "," << mCommand.cutSlice.mBladePositionXY.ZZ / mm << ")";
		*fileHandle << "**************************************************************\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

void Sequence::Commandline::printParameters() const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mCommand.moveStage.mSliceNumber << "\n";
		std::cout << "Stack ij = (" << mCommand.moveStage.mStackIJ.II << "," << mCommand.moveStage.mStackIJ.JJ << ")\n";
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.mStackCenterXY.XX / mm << "," << mCommand.moveStage.mStackCenterXY.YY / mm << ")\n\n";
		break;
	case Action::ID::ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength (nm) = " << mCommand.acqStack.mWavelength_nm << "\n";
		std::cout << "scanDirZ = " << mCommand.acqStack.mScanDirZ << "\n";
		std::cout << "scanZi (mm) / stackDepth (mm) = " << mCommand.acqStack.mScanZi / mm << "/" << mCommand.acqStack.mStackDepth << "\n";
		std::cout << "scanPi (mW) / stackPdiff (mW/um) = " << mCommand.acqStack.mScanPi / mW << "/" << mCommand.acqStack.mStackPinc / mWpum << "\n\n";
		break;
	case Action::ID::SAV:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	case Action::ID::CUT:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

std::string Sequence::Commandline::actionToString_(const Action::ID action) const
{
	switch (action)
	{
	case Action::ID::CUT:
		return "CUT";
	case Action::ID::SAV:
		return "SAV";
	case Action::ID::ACQ:
		return "ACQ";
	case Action::ID::MOV:
		return "MOV";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequence"
//Constructor using the sample's ROI. The number of stacks is calculated automatically based on the FFOV
Sequence::Sequence(const Sample sample, const Stack stack) : mSample{ sample }, mStack{ stack }
{
	//Initialize the z-stage with the position of the surface of the sample
	mScanZi = mSample.mSurfaceZ;

	//Initialize the height of the plane to slice
	mPlaneToSliceZ = mScanZi + mStack.mDepth - mSample.mCutAboveBottomOfStack;

	//Calculate the number of stacks in the axis STAGEX and STAGEY based on the sample size and stack FFOV
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
	mStackArrayDimIJ.II = static_cast<int>(std::ceil(  1/(1- mStack.mOverlap_frac.XX) * (mSample.mSizeRequest.XX / mStack.mFFOV.XX - 1) + 1  ));		//Number of stacks in the axis STAGEX
	mStackArrayDimIJ.JJ = static_cast<int>(std::ceil(  1/(1- mStack.mOverlap_frac.YY) * (mSample.mSizeRequest.YY / mStack.mFFOV.YY - 1) + 1  ));		//Number of stacks in the axis STAGEY

	//Calculate the effective ROI covered by the stacks, which might be slightly larger than the sample's ROI
	mROIeff.XMIN = mSample.mROIrequest.XMIN;
	mROIeff.YMIN = mSample.mROIrequest.YMIN;
	mROIeff.XMAX = mSample.mROIrequest.XMIN + mStack.mFFOV.XX  * ((1 - mStack.mOverlap_frac.XX) * mStackArrayDimIJ.II + 0.5);
	mROIeff.YMAX = mSample.mROIrequest.YMIN + mStack.mFFOV.YY  * ((1 - mStack.mOverlap_frac.YY) * mStackArrayDimIJ.JJ + 0.5);

	//Calculate the total number of stacks in a vibratome slice and in the entire sample
	mNtotalSlices = static_cast<int>(std::ceil(  1 / (1 - mStack.mOverlap_frac.ZZ) * (mSample.mSizeRequest.ZZ / mStack.mDepth - 1) + 1  ));	//Total number of slices in the entire sample
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.II * mStackArrayDimIJ.JJ };													//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * mNtotalStacksPerVibratomeSlice };						//Total number of stacks in the entire sample

	//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Check that cutAboveBottomOfStack is greater than the stack z-overlap
	const double stackOverlapZ{ mStack.mOverlap_frac.ZZ * mStack.mDepth };	//Dummy local variable
	if (mSample.mCutAboveBottomOfStack < stackOverlapZ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'cutAboveBottomOfStack' must be greater than the stack z-overlap " + toString(stackOverlapZ / um, 1) + " um");

}

//Constructor using the initial stack center and number of stacks. To be used without slicing
Sequence::Sequence(Sample sample, const Stack stack, const POSITION2 stackCenterXY, const INDICES2 stackArrayDimIJ) : mSample{ sample }, mStack{ stack }, mStackArrayDimIJ{ stackArrayDimIJ }
{
	if (stackArrayDimIJ.II <= 0 || stackArrayDimIJ.JJ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack array dimension must be >=1");

	//Calculate the effective ROI covered by the stacks
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 )
	mROIeff.XMIN = stackCenterXY.XX - mStack.mFFOV.XX / 2;
	mROIeff.YMIN = stackCenterXY.YY - mStack.mFFOV.YY / 2;
	mROIeff.XMAX = mROIeff.XMIN + mStack.mFFOV.XX * ((1 - mStack.mOverlap_frac.XX) * (mStackArrayDimIJ.II - 1) + 1);
	mROIeff.YMAX = mROIeff.YMIN + mStack.mFFOV.YY * ((1 - mStack.mOverlap_frac.YY) * (mStackArrayDimIJ.JJ - 1) + 1);

	//Initialize the z-stage with the position of the surface of the sample
	mScanZi = mSample.mSurfaceZ;

	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.II * mStackArrayDimIJ.JJ };								//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * mNtotalStacksPerVibratomeSlice };	//Total number of stacks in the entire sample. mNtotalSlices is fixed at 1

	//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Set these unused parameters to 0 to avoid any confusion when printing the parameters to text
	mSample.mROIrequest = { 0,0,0,0 };
	mSample.mSizeRequest = { 0,0,0 };
}

Sequence::Commandline Sequence::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

//Generate a scan pattern and use the vibratome to slice the sample
void Sequence::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";

	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

		for (std::vector<int>::size_type iterWL = 0; iterWL != mSample.mFluorLabelList.size(); iterWL++)
		{
			//STAGEY sits under the other 2 stages and, therefore, is the slowest to react. For the best performance, iterate over STAGEX often and over STAGEY less often
			while (JJ >= 0 && JJ < mStackArrayDimIJ.JJ)			//STAGEY direction
			{
				while (II >= 0 && II < mStackArrayDimIJ.II)		//STAGEX direction
				{
					moveStage_({ II, JJ });
					acqStack_(iterWL);
					saveStack_();
					II += mScanDir.at(Stage::XX);		//Increase the iterator in the axis STAGE X
				}

				//Initialize the next cycle by going back in the axis STAGEX one step and switching the scanning direction
				II -= mScanDir.at(Stage::XX);
				reverseStageScanDirection_(Stage::XX);
				JJ += mScanDir.at(Stage::YY);	//Increase the iterator in the axis STAGEY
			}
			//Initialize the next cycle by going back in the axis STAGEY one step and switching the scanning direction
			JJ -= mScanDir.at(Stage::YY);
			reverseStageScanDirection_(Stage::YY);
		}

		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

//Like generateCommandList() but instead of a list of command, Generate a list of locations to be called by frameByFrameZscanTilingXY()
//Note that this sequence does not use the vibratome
std::vector<POSITION2> Sequence::generateLocationList()
{
	std::vector<POSITION2> locationList;
	//std::cout << "Generating the location list..." << "\n";

	int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
	resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

	//The y-stage is the slowest to react because it sits under of other 2 stages. For the best performance, iterate over STAGEX often and over STAGEY less often
	while (JJ >= 0 && JJ < mStackArrayDimIJ.JJ)			//STAGEY direction
	{
		while (II >= 0 && II < mStackArrayDimIJ.II)		//STAGEX direction
		{
			const POSITION2 stackCenterXY{ stackIndicesToStackCenter_({ II, JJ }) };
			locationList.push_back(stackCenterXY);

			//std::cout << "x = " << stackCenterXY.at(X) / mm << "\ty = " << stackCenterXY.at(Y) / mm << "\n";		//For debugging
			II += mScanDir.at(Stage::XX);		//Increase the iterator in the axis STAGEX
		}

		//Initialize the next cycle by going back in the axis STAGEX one step and switching the scanning direction
		II -= mScanDir.at(Stage::XX);
		reverseStageScanDirection_(Stage::XX);
		JJ += mScanDir.at(Stage::YY);	//Increase the iterator in the axis STAGEY
	}
	//Initialize the next cycle by going back in the axis STAGEY one step and switching the scanning direction
	JJ -= mScanDir.at(Stage::YY);
	reverseStageScanDirection_(Stage::YY);

	return locationList;
}

int Sequence::size() const
{
	return mCommandCounter;
}

Stack Sequence::stack() const
{
	return mStack;
}

void Sequence::printSequenceParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan direction {STAGEX,STAGEY, STAGEZ} = {" << mInitialScanDir.at(Stage::XX) << ", " << mInitialScanDir.at(Stage::YY) << ", " << mInitialScanDir.at(Stage::ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Effective ROI [YMIN, XMIN, YMAX, XMAX] (mm) = [" << mROIeff.YMIN / mm << ", " << mROIeff.XMIN / mm << ", " << mROIeff.YMAX / mm << ", " << mROIeff.XMAX / mm << "]\n";
	*fileHandle << "Effective sample size (STAGEX, STAGEY, STAGEZ) (mm) = (" << effectiveSize_().XX / mm << ", " << effectiveSize_().YY / mm << ", " << effectiveSize_().ZZ / mm << ")\n";
	*fileHandle << "Sample surface position STAGEZ (mm) = " << mSample.mSurfaceZ / mm << "\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "Total # tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (STAGEX,STAGEY) = (" << mStackArrayDimIJ.II << ", " << mStackArrayDimIJ.JJ << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";
	*fileHandle << "\n";
}

//Print the commandlist to file
void Sequence::printToFile(const std::string fileName) const
{
	std::ofstream *fileHandle{ new std::ofstream(folderPath + fileName + ".txt") };

	*fileHandle << std::fixed;	//Show a fixed number of digits

	mSample.printParams(fileHandle);
	mSample.mFluorLabelList.printParams(fileHandle);
	printSequenceParams(fileHandle);
	mStack.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
		//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mStackArrayDimIJ.at(X) * mStackArrayDimIJ.at(Y)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}

//The initial laser power for scanning a stack depends on whether the stack is imaged from the top down or from the bottom up.
double Sequence::calculateStackInitialPower_(const double Ptop, const double stackPinc, const int scanDirZ, const double stackDepth)
{
	if (scanDirZ > 0)	//z-stage moves up for top-down imaging
		return Ptop;
	else				//z-stage moves down for bottom-up imaging
		return Ptop + stackDepth * stackPinc;
}

//The first stack center is L/2 away from the ROI's edge. The next center is at (1-a)*L away from the first center, where a*L is the stack overlap
POSITION2 Sequence::stackIndicesToStackCenter_(const INDICES2 stackArrayIndicesIJ) const
{
	const double overlapX_frac{ mStack.mOverlap_frac.XX };
	const double overlapY_frac{ mStack.mOverlap_frac.YY };

	POSITION2 stagePositionXY;
	stagePositionXY.XX = mROIeff.XMIN + mStack.mFFOV.XX  * ((1 - overlapX_frac) * stackArrayIndicesIJ.II + 0.5);
	stagePositionXY.YY = mROIeff.YMIN + mStack.mFFOV.YY  * ((1 - overlapY_frac) * stackArrayIndicesIJ.JJ + 0.5);

	return stagePositionXY;
}

void Sequence::reverseStageScanDirection_(const Stage::Axis axis)
{
	switch (axis)
	{
	case Stage::XX:
		mScanDir.at(Stage::XX) *= -1;
		break;
	case Stage::YY:
		mScanDir.at(Stage::YY) *= -1;
		break;
	case Stage::ZZ:
		mScanDir.at(Stage::ZZ) *= -1;
		break;
	}
}

void Sequence::resetStageScanDirections_()
{
	mScanDir = mInitialScanDir;
}

//Convert a ROI = {ymin, xmin, ymax, xmax} to the equivalent sample size in the axis STAGEX and STAGEY
SAMPLESIZE3 Sequence::effectiveSize_() const
{
	return { mROIeff.XMAX - mROIeff.XMIN, mROIeff.YMAX - mROIeff.YMIN, mStack.mDepth * ((1 - mStack.mOverlap_frac.ZZ) * (mNtotalSlices - 1) + 1) };
}

void Sequence::moveStage_(const INDICES2 stackIJ)
{
	const POSITION2 stackCenterXY = stackIndicesToStackCenter_(stackIJ);

	Commandline commandline;
	commandline.mAction = Action::ID::MOV;
	commandline.mCommand.moveStage = { mSliceCounter, stackIJ, stackCenterXY };

	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequence::acqStack_(const int iterWL)
{
	//Read the corresponding laser configuration
	const FluorLabelList::FluorLabel fluorLabel{ mSample.mFluorLabelList.at(iterWL) };

	//Determine if the initial laser power is the lowest (top of the stack) or the highest (bottom of the stack)
	const double scanPi = calculateStackInitialPower_(fluorLabel.mScanPi, fluorLabel.mStackPinc, mScanDir.at(Stage::ZZ), mStack.mDepth);

	Commandline commandline;
	commandline.mAction = Action::ID::ACQ;
	commandline.mCommand.acqStack = { mStackCounter, fluorLabel.mWavelength_nm, mScanDir.at(Stage::ZZ), mScanZi, mStack.mDepth, scanPi, fluorLabel.mStackPinc };

	mCommandList.push_back(commandline);
	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	mScanZi += mScanDir.at(Stage::ZZ) * mStack.mDepth;		//Next initial z-scan position
	reverseStageScanDirection_(Stage::ZZ);					//Switch the scanning direction in the axis STAGEZ
}

void Sequence::saveStack_()
{
	Commandline commandline;
	commandline.mAction = Action::ID::SAV;

	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequence::cutSlice_()
{
	//Move the sample to face the vibratome blade. Note the additional offset in the axis STAGEZ
	const POSITION3 samplePositionXYZ{ mSample.mBladePositionXY.XX, mSample.mBladePositionXY.YY, mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline;
	commandline.mAction = Action::ID::CUT;
	commandline.mCommand.cutSlice = { samplePositionXYZ };

	mCommandList.push_back(commandline);
	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the z-stage for the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlap_frac.ZZ) * mStack.mDepth;

	//Increase the height of the plane to be cut in the next iteration
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mPlaneToSliceZ += (1 - mStack.mOverlap_frac.ZZ) * mStack.mDepth;
}
#pragma endregion "sequencer"