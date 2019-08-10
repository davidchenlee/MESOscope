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
Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<double2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList) :
	mName{ sampleName }, mImmersionMedium{ immersionMedium }, mObjectiveCollar{ objectiveCollar }, mStageSoftPosLimXYZ{ stageSoftPosLimXYZ }, mFluorLabelList{ fluorLabelList }{}

Sample::Sample(const Sample& sample, ROI roi, const double sampleLengthZ, const double sampleSurfaceZ, const double sliceOffset) :
	mName{ sample.mName }, mImmersionMedium{ sample.mImmersionMedium }, mObjectiveCollar{ sample.mObjectiveCollar }, mFluorLabelList{ sample.mFluorLabelList }, mROIrequest{ roi }, mSurfaceZ{ sampleSurfaceZ }, mCutAboveBottomOfStack{ sliceOffset }
{
	//Convert input ROI = {ymin, xmin, ymax, xmax} to the equivalent sample length in the axis STAGEX and STAGEY
	mSizeRequest.at(Stage::X) = mROIrequest.at(XMAX) - mROIrequest.at(XMIN);
	mSizeRequest.at(Stage::Y) = mROIrequest.at(YMAX) - mROIrequest.at(YMIN);
	mSizeRequest.at(Stage::Z) = sampleLengthZ;

	if (mSizeRequest.at(Stage::X) <= 0 || mSizeRequest.at(Stage::Y) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

	if (mSizeRequest.at(Stage::Z) <= 0)
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
	*fileHandle << "Requested ROI [STAGEYmin, STAGEXmin, STAGEYmax, STAGEXmax] (mm) = [" << mROIrequest.at(YMIN) / mm << ", " << mROIrequest.at(XMIN) / mm << ", " << mROIrequest.at(YMAX) / mm << ", " << mROIrequest.at(XMAX) / mm << "]\n";
	*fileHandle << "Requested sample size (STAGEX, STAGEY, STAGEZ) (mm) = (" << mSizeRequest.at(Stage::X) / mm << ", " << mSizeRequest.at(Stage::Y) / mm << ", " << mSizeRequest.at(Stage::Z) / mm << ")\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Blade position (STAGEX, STAGEY) (mm) = (" << mBladePositionXY.at(Stage::X) / mm << ", " << mBladePositionXY.at(Stage::Y) / mm << ")\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset (um) = " << mBladeFocalplaneOffsetZ / um << "\n";
	*fileHandle << "Cut above the bottom of the stack (um) = " << mCutAboveBottomOfStack / um << "\n";
	*fileHandle << "\n";
}
#pragma endregion "Sample"

#pragma region "Stack"
Stack::Stack(const double2 FFOV, const double stepSizeZ, const int nFrames, const double3 overlap_frac) :
	mFFOV{ FFOV }, mStepSizeZ{ stepSizeZ }, mDepth{ stepSizeZ *  nFrames }, mOverlap_frac{ overlap_frac }
{
	if (FFOV.at(Stage::X) <= 0 || FFOV.at(Stage::Y) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

	if (mStepSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

	if (mDepth <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

	if (mOverlap_frac.at(Stage::X) < 0 || mOverlap_frac.at(Stage::Y) < 0 || mOverlap_frac.at(Stage::Z) < 0
		|| mOverlap_frac.at(Stage::X) > 0.2 || mOverlap_frac.at(Stage::Y) > 0.2 || mOverlap_frac.at(Stage::Z) > 0.2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-0.2]");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (STAGEX, STAGEY) (um) = (" << mFFOV.at(Stage::X) / um << ", " << mFFOV.at(Stage::Y) / um << ")\n";
	*fileHandle << "Step size Z (um) = " << mStepSizeZ / um << "\n";
	*fileHandle << "Stack depth (um) = " << mDepth / um << "\n";
	*fileHandle << std::setprecision(2);
	*fileHandle << "Stack overlap (frac) = (" << mOverlap_frac.at(Stage::X) << ", " << mOverlap_frac.at(Stage::Y) << ", " << mOverlap_frac.at(Stage::Z) << ")\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Stack overlap (um) = (" << mOverlap_frac.at(Stage::X) * mFFOV.at(Stage::X) / um << ", " << mOverlap_frac.at(Stage::Y) * mFFOV.at(Stage::Y) / um << ", " << mOverlap_frac.at(Stage::Z) * mDepth << ")\n";
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
		*fileHandle << "\t(" << mCommand.moveStage.mStackIJ.at(Stage::X) << "," << mCommand.moveStage.mStackIJ.at(Stage::Y) << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mCommand.moveStage.mStackCenterXY.at(Stage::X) / mm << "," << mCommand.moveStage.mStackCenterXY.at(Stage::Y) / mm << ")\n";
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
		*fileHandle << "(" << mCommand.cutSlice.mBladePositionXY.at(Stage::X) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(Stage::Y) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(Stage::Z) / mm << ")";
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
		std::cout << "Stack ij = (" << mCommand.moveStage.mStackIJ.at(Stage::X) << "," << mCommand.moveStage.mStackIJ.at(Stage::Y) << ")\n";
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.mStackCenterXY.at(Stage::X) / mm << "," << mCommand.moveStage.mStackCenterXY.at(Stage::Y) / mm << ")\n\n";
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
	mStackArrayDimIJ.at(Stage::X) = static_cast<int>(std::ceil(  1/(1- mStack.mOverlap_frac.at(Stage::X)) * (mSample.mSizeRequest.at(Stage::X) / mStack.mFFOV.at(Stage::X) - 1) + 1  ));		//Number of stacks in the axis STAGEX
	mStackArrayDimIJ.at(Stage::Y) = static_cast<int>(std::ceil(  1/(1- mStack.mOverlap_frac.at(Stage::Y)) * (mSample.mSizeRequest.at(Stage::Y) / mStack.mFFOV.at(Stage::Y) - 1) + 1  ));		//Number of stacks in the axis STAGEY

	//Calculate the effective ROI covered by the stacks, which might be slightly larger than the sample's ROI
	mROIeffective.at(XMIN) = mSample.mROIrequest.at(XMIN);
	mROIeffective.at(YMIN) = mSample.mROIrequest.at(YMIN);
	mROIeffective.at(XMAX) = mSample.mROIrequest.at(XMIN) + mStack.mFFOV.at(Stage::X)  * ((1 - mStack.mOverlap_frac.at(Stage::X)) * mStackArrayDimIJ.at(Stage::X) + 0.5);
	mROIeffective.at(YMAX) = mSample.mROIrequest.at(YMIN) + mStack.mFFOV.at(Stage::Y)  * ((1 - mStack.mOverlap_frac.at(Stage::Y)) * mStackArrayDimIJ.at(Stage::Y) + 0.5);

	//Calculate the total number of stacks in a vibratome slice and in the entire sample
	mNtotalSlices = static_cast<int>(std::ceil(  1 / (1 - mStack.mOverlap_frac.at(Stage::Z)) * (mSample.mSizeRequest.at(Stage::Z) / mStack.mDepth - 1) + 1  ));		//Total number of slices in the entire sample
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(Stage::X) * mStackArrayDimIJ.at(Stage::Y) };														//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * mNtotalStacksPerVibratomeSlice };						//Total number of stacks in the entire sample

	//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Check that cutAboveBottomOfStack is greater than the stack z-overlap
	const double stackOverlapZ{ mStack.mOverlap_frac.at(Stage::Z) * mStack.mDepth };	//Dummy local variable
	if (mSample.mCutAboveBottomOfStack < stackOverlapZ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'cutAboveBottomOfStack' must be greater than the stack z-overlap " + toString(stackOverlapZ / um, 1) + " um");

}

//Constructor using the initial stack center and number of stacks. To be used without slicing
Sequence::Sequence(Sample sample, const Stack stack, const double2 stackCenterXY, const int2 stackArrayDimIJ) : mSample{ sample }, mStack{ stack }, mStackArrayDimIJ{ stackArrayDimIJ }
{
	if (stackArrayDimIJ.at(Stage::X) <= 0 || stackArrayDimIJ.at(Stage::Y) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack array dimension must be >=1");

	//Calculate the effective ROI covered by the stacks
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 )
	mROIeffective.at(XMIN) = stackCenterXY.at(Stage::X) - mStack.mFFOV.at(Stage::X) / 2;
	mROIeffective.at(YMIN) = stackCenterXY.at(Stage::Y) - mStack.mFFOV.at(Stage::Y) / 2;
	mROIeffective.at(XMAX) = mROIeffective.at(XMIN) + mStack.mFFOV.at(Stage::X) * ((1 - mStack.mOverlap_frac.at(Stage::X)) * (mStackArrayDimIJ.at(Stage::X) - 1) + 1);
	mROIeffective.at(YMAX) = mROIeffective.at(YMIN) + mStack.mFFOV.at(Stage::Y) * ((1 - mStack.mOverlap_frac.at(Stage::Y)) * (mStackArrayDimIJ.at(Stage::Y) - 1) + 1);

	//Initialize the z-stage with the position of the surface of the sample
	mScanZi = mSample.mSurfaceZ;

	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(Stage::X) * mStackArrayDimIJ.at(Stage::Y) };									//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * mNtotalStacksPerVibratomeSlice };	//Total number of stacks in the entire sample. mNtotalSlices is fixed at 1

	//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Set these unused parameters to 0 to avoid any confusion when printing the parameters to text
	mSample.mROIrequest = { 0,0,0,0 };
	mSample.mSizeRequest = { 0,0,0 };

	polyMask.reserve(mStackArrayDimIJ.at(Stage::X) * mStackArrayDimIJ.at(Stage::Y));
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
			//The STAGEY sits under the other 2 stages and therefore is slowest to react. For the best performance, iterate over STAGEX often and over STAGEY less often
			while (JJ >= 0 && JJ < mStackArrayDimIJ.at(Stage::Y))			//STAGEY direction
			{
				while (II >= 0 && II < mStackArrayDimIJ.at(Stage::X))		//STAGEX direction
				{
					moveStage_({ II, JJ });
					acqStack_(iterWL);
					saveStack_();
					II += mScanDir.at(Stage::X);		//Increase the iterator in the axis STAGE X
				}

				//Initialize the next cycle by going back in the axis STAGEX one step and switching the scanning direction
				II -= mScanDir.at(Stage::X);
				reverseStageScanDirection_(Stage::X);
				JJ += mScanDir.at(Stage::Y);	//Increase the iterator in the axis STAGEY
			}
			//Initialize the next cycle by going back in the axis STAGEY one step and switching the scanning direction
			JJ -= mScanDir.at(Stage::Y);
			reverseStageScanDirection_(Stage::Y);
		}

		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

//Like generateCommandList() but instead of a list of command, Generate a list of locations to be called by frameByFrameZscanTilingXY()
//Note that this sequence does not use the vibratome
std::vector<double2> Sequence::generateLocationList()
{
	std::vector<double2> locationList;
	//std::cout << "Generating the location list..." << "\n";

	int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
	resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

	//The y-stage is the slowest to react because it sits under of other 2 stages. For the best performance, iterate over STAGEX often and over STAGEY less often
	while (JJ >= 0 && JJ < mStackArrayDimIJ.at(Stage::Y))			//STAGEY direction
	{
		while (II >= 0 && II < mStackArrayDimIJ.at(Stage::X))		//STAGEX direction
		{
			const double2 stackCenterXY{ stackIndicesToStackCenter_({ II, JJ }) };
			locationList.push_back(stackCenterXY);

			//std::cout << "x = " << stackCenterXY.at(X) / mm << "\ty = " << stackCenterXY.at(Y) / mm << "\n";		//For debugging
			II += mScanDir.at(Stage::X);		//Increase the iterator in the axis STAGEX
		}

		//Initialize the next cycle by going back in the axis STAGEX one step and switching the scanning direction
		II -= mScanDir.at(Stage::X);
		reverseStageScanDirection_(Stage::X);
		JJ += mScanDir.at(Stage::Y);	//Increase the iterator in the axis STAGEY
	}
	//Initialize the next cycle by going back in the axis STAGEY one step and switching the scanning direction
	JJ -= mScanDir.at(Stage::Y);
	reverseStageScanDirection_(Stage::Y);

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
	*fileHandle << "Stages initial scan direction {STAGEX,STAGEY, STAGEZ} = {" << mInitialScanDir.at(Stage::X) << ", " << mInitialScanDir.at(Stage::Y) << ", " << mInitialScanDir.at(Stage::Z) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Effective ROI [YMIN, XMIN, YMAX, XMAX] (mm) = [" << mROIeffective.at(YMIN) / mm << ", " << mROIeffective.at(XMIN) / mm << ", " << mROIeffective.at(YMAX) / mm << ", " << mROIeffective.at(XMAX) / mm << "]\n";
	*fileHandle << "Effective sample size (STAGEX, STAGEY, STAGEZ) (mm) = (" << effectiveSize_().at(Stage::X) / mm << ", " << effectiveSize_().at(Stage::Y) / mm << ", " << effectiveSize_().at(Stage::Z) / mm << ")\n";
	*fileHandle << "Sample surface position STAGEZ (mm) = " << mSample.mSurfaceZ / mm << "\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "Total # tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (STAGEX,STAGEY) = (" << mStackArrayDimIJ.at(Stage::X) << ", " << mStackArrayDimIJ.at(Stage::Y) << ")\n";
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
double2 Sequence::stackIndicesToStackCenter_(const int2 stackArrayIndicesIJ) const
{
	const double overlapX_frac{ mStack.mOverlap_frac.at(Stage::X) };
	const double overlapY_frac{ mStack.mOverlap_frac.at(Stage::Y) };

	double2 stagePositionXY;
	stagePositionXY.at(Stage::X) = mROIeffective.at(XMIN) + mStack.mFFOV.at(Stage::X)  * ((1 - overlapX_frac) * stackArrayIndicesIJ.at(Stage::X) + 0.5);
	stagePositionXY.at(Stage::Y) = mROIeffective.at(YMIN) + mStack.mFFOV.at(Stage::Y)  * ((1 - overlapY_frac) * stackArrayIndicesIJ.at(Stage::Y) + 0.5);

	return stagePositionXY;
}

void Sequence::reverseStageScanDirection_(const Stage::Axis axis)
{
	switch (axis)
	{
	case Stage::X:
		mScanDir.at(Stage::X) *= -1;
		break;
	case Stage::Y:
		mScanDir.at(Stage::Y) *= -1;
		break;
	case Stage::Z:
		mScanDir.at(Stage::Z) *= -1;
		break;
	}
}

void Sequence::resetStageScanDirections_()
{
	mScanDir = mInitialScanDir;
}

//Convert ia ROI = {ymin, xmin, ymax, xmax} to the equivalent sample size in the axis STAGEX and STAGEY
double3 Sequence::effectiveSize_() const
{
	return { mROIeffective.at(XMAX) - mROIeffective.at(XMIN), mROIeffective.at(YMAX) - mROIeffective.at(YMIN), mStack.mDepth * ((1 - mStack.mOverlap_frac.at(Stage::Z)) * (mNtotalSlices - 1) + 1) };
}

void Sequence::moveStage_(const int2 stackIJ)
{
	const double2 stackCenterXY = stackIndicesToStackCenter_(stackIJ);

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
	const double scanPi = calculateStackInitialPower_(fluorLabel.mScanPi, fluorLabel.mStackPinc, mScanDir.at(Stage::Z), mStack.mDepth);

	Commandline commandline;
	commandline.mAction = Action::ID::ACQ;
	commandline.mCommand.acqStack = { mStackCounter, fluorLabel.mWavelength_nm, mScanDir.at(Stage::Z), mScanZi, mStack.mDepth, scanPi, fluorLabel.mStackPinc };

	mCommandList.push_back(commandline);
	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	mScanZi += mScanDir.at(Stage::Z) * mStack.mDepth;		//Next initial z-scan position
	reverseStageScanDirection_(Stage::Z);					//Switch the scanning direction in the axis STAGEZ
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
	const double3 samplePositionXYZ{ mSample.mBladePositionXY.at(Stage::X), mSample.mBladePositionXY.at(Stage::Y), mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline;
	commandline.mAction = Action::ID::CUT;
	commandline.mCommand.cutSlice = { samplePositionXYZ };

	mCommandList.push_back(commandline);
	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the z-stage for the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlap_frac.at(Stage::Z)) * mStack.mDepth;

	//Increase the height of the plane to be cut in the next iteration
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mPlaneToSliceZ += (1 - mStack.mOverlap_frac.at(Stage::Z)) * mStack.mDepth;
}

void Sequence::generatePolyMask_(const std::vector<double2> vertices)
{
	for (std::vector<int>::size_type II = 0; II != mStackArrayDimIJ.at(Stage::X); II++)
		for (std::vector<int>::size_type JJ = 0; JJ != mStackArrayDimIJ.at(Stage::Y); JJ++)
			polyMask.push_back(0);
}

//maskIJ[I][J] = 0 or 1
void Sequence::findContour_(const bool **maskIJ) const
{
	std::vector<int2> minmaxII, minmaxJJ;

	//for every II, calculate the max and min indices JJ that satisfy maskIJ[II][JJ] == true
	for (int II = 0; II < mStackArrayDimIJ.at(Stage::X); II++)		//STAGEX direction
	{
		std::vector<int> JJs;
		for (int JJ = 0; JJ < mStackArrayDimIJ.at(Stage::Y); JJ++)	//STAGEY direction
			if (maskIJ[II][JJ] == true)
				JJs.push_back(JJ);			//collect all the JJs that satisfy masjIJ[II][JJ] == true

		minmaxJJ.push_back({ JJs.front(), JJs.back() });// {min JJ, max JJ}
	}

	// {II, {min JJ, max JJ} }

	//for every JJ, calculate the max and min indices II that satisfy maskIJ[II][JJ] == true
	for (int JJ = 0; JJ < mStackArrayDimIJ.at(Stage::Y); JJ++)		//STAGEY direction
	{
		std::vector<int> IIs;
		for (int II = 0; II < mStackArrayDimIJ.at(Stage::X); II++)	//STAGEX direction
			if (maskIJ[II][JJ] == true)
				IIs.push_back(JJ);			//collect all the JJs that satisfy masjIJ[II][JJ] == true

		minmaxJJ.push_back({ IIs.front(), IIs.back() });// {min II, max II}
	}
	// {{min II, max II}, JJ }

}
#pragma endregion "sequencer"