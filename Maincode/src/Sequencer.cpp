#include "Sequencer.h"

#pragma region "FluorLabelList"
FluorLabelList::FluorLabelList(const std::vector<FluorLabel> fluorLabelList) :
	mFluorLabelList{ fluorLabelList }
{}

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
	*fileHandle << "LASERS ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mFluorLabelList.size(); iterWL++)
	{
		*fileHandle << "Wavelength = " << mFluorLabelList.at(iterWL).mWavelength_nm <<
			" nm\nPower = " << mFluorLabelList.at(iterWL).mScanPmin / mW <<
			" mW\nPower increase = " << mFluorLabelList.at(iterWL).mStackPinc / mWpum << " mW/um\n";
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
	mName{ sampleName },
	mImmersionMedium{ immersionMedium },
	mObjectiveCollar{ objectiveCollar },
	mStageSoftPosLimXYZ{ stageSoftPosLimXYZ },
	mFluorLabelList{ fluorLabelList }
{}

Sample::Sample(const Sample& sample, const POSITION2 centerXY, const SAMPLESIZE3 sizeXYZ, const double sampleSurfaceZ, const double sliceOffset) :
	mName{ sample.mName },
	mImmersionMedium{ sample.mImmersionMedium },
	mObjectiveCollar{ sample.mObjectiveCollar },
	mFluorLabelList{ sample.mFluorLabelList },
	mCenterXY{ centerXY },
	mSizeRequest{ sizeXYZ },
	mSurfaceZ{ sampleSurfaceZ },
	mCutAboveBottomOfStack{ sliceOffset }
{
	if (mSizeRequest.XX <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length X must be >0");

	if (mSizeRequest.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Y must be >0");

	if (mCutAboveBottomOfStack < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be >=0");
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
	*fileHandle << "Sample center (stageX, stageY, stageZ) = (" << mSizeRequest.XX / mm << " mm, " << mSizeRequest.YY / mm << " mm, " << mSizeRequest.ZZ / mm << " mm)\n";
	*fileHandle << "Requested sample size (stageX, stageY, stageZ) = (" << mSizeRequest.XX / mm << " mm, " << mSizeRequest.YY / mm << " mm, " << mSizeRequest.ZZ / mm << " mm)\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Blade position (stageX, stageY) = (" << mBladePositionXY.XX / mm << " mm, " << mBladePositionXY.YY / mm << " mm)\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset = " << mBladeFocalplaneOffsetZ / um << " um\n";
	*fileHandle << "Cut above the bottom of the stack = " << mCutAboveBottomOfStack / um << " um\n";
	*fileHandle << "\n";
}
#pragma endregion "Sample"

#pragma region "Stack"
Stack::Stack(const FFOV2 FFOV, const double stepSizeZ, const int nFrames, const TILEOVERLAP4 overlap_frac) :
	mFFOV{ FFOV },
	mStepSizeZ{ stepSizeZ },
	mDepth{ stepSizeZ *  nFrames },
	mOverlap_frac{ overlap_frac }
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

	if (mStepSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The Z-stage step size must be positive");

	if (mDepth <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

	if (mOverlap_frac.XX < 0 || mOverlap_frac.YY < 0 || mOverlap_frac.XX > 0.2 || mOverlap_frac.YY > 0.2 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in XY must be in the range [0-0.2]");

	if (mOverlap_frac.ZZ < 0 || mOverlap_frac.ZZ > 0.5)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in Z must be in the range [0-0.5]");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (stageX, stageY) = (" << mFFOV.XX / um << " um, " << mFFOV.YY / um << " um)\n";
	*fileHandle << "Step size Z = " << mStepSizeZ / um << " um\n";
	*fileHandle << "Stack depth = " << mDepth / um << " um\n";
	*fileHandle << std::setprecision(2);
	*fileHandle << "Stack overlap = (" << mOverlap_frac.XX << ", " << mOverlap_frac.YY << ", " << mOverlap_frac.ZZ << ") (frac)\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Stack overlap = (" << mOverlap_frac.XX * mFFOV.XX / um << " um, " << mOverlap_frac.YY * mFFOV.YY / um << " um, " << mOverlap_frac.ZZ * mDepth << " um)\n";
	*fileHandle << "\n";
}
#pragma endregion "Stack"

#pragma region "Commandline"
Sequencer::Commandline::Commandline(const Action::ID action) :
	mAction{ action }
{}

std::string Sequencer::Commandline::printHeader() const
{
	return 	"Action\tSlice#\tTileIJ\t(stageX,stageY)\tStack#\tWavlen\tDirZ\tstageZ<\tstageZ>\tP<\tP>";
}

std::string Sequencer::Commandline::printHeaderUnits() const
{
	return "\t\t(mm,mm)\t\t\tnm\t\tmm\tmm\tmW\tmW";
}

void Sequencer::Commandline::printToFile(std::ofstream *fileHandle) const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mParam.moveStage.mSliceNumber;
		*fileHandle << "\t(" << mParam.moveStage.mTilesIJ.II << "," << mParam.moveStage.mTilesIJ.JJ << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mParam.moveStage.mTileCenterXY.XX / mm << "," << mParam.moveStage.mTileCenterXY.YY / mm << ")\n";
		break;
	case Action::ID::ACQ:
		*fileHandle << actionToString_(mAction) << "\t\t\t\t\t";
		*fileHandle << mParam.acqStack.mStackNumber << "\t";
		*fileHandle << mParam.acqStack.mWavelength_nm << "\t";
		*fileHandle << SCANDIRtoInt(mParam.acqStack.mScanDirZ) << "\t";
		*fileHandle << std::setprecision(3);
		*fileHandle << mParam.acqStack.mScanZmin / mm << "\t";
		*fileHandle << (mParam.acqStack.mScanZmin + mParam.acqStack.mStackDepth) / mm << "\t";
		*fileHandle << std::setprecision(0);
		*fileHandle << mParam.acqStack.mScanPmin << "\t" << mParam.acqStack.mScanPmin + mParam.acqStack.mStackDepth * mParam.acqStack.mStackPinc << "\n";
		break;
	case Action::ID::SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case Action::ID::CUT:
		*fileHandle << actionToString_(mAction);
		*fileHandle << std::setprecision(3);
		*fileHandle << "\t************Sample facing the vibratome at = ";
		*fileHandle << "(" << mParam.cutSlice.mBladePositionXY.XX / mm << "," << mParam.cutSlice.mBladePositionXY.YY / mm << "," << mParam.cutSlice.mBladePositionXY.ZZ / mm << ") mm";
		*fileHandle << "**************************************************************\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

void Sequencer::Commandline::printParameters() const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mParam.moveStage.mSliceNumber << "\n";
		std::cout << "Tile ij = (" << mParam.moveStage.mTilesIJ.II << "," << mParam.moveStage.mTilesIJ.JJ << ")\n";
		std::cout << "Tile center (stageX, stageY) = (" << mParam.moveStage.mTileCenterXY.XX / mm << "," << mParam.moveStage.mTileCenterXY.YY / mm << ") mm\n\n";
		break;
	case Action::ID::ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength = " << mParam.acqStack.mWavelength_nm << " nm\n";
		std::cout << "scanDirZ = " << SCANDIRtoInt(mParam.acqStack.mScanDirZ) << "\n";
		std::cout << "scanZmin / stackDepth = " << mParam.acqStack.mScanZmin / mm << " mm/" << mParam.acqStack.mStackDepth << " mm\n";
		std::cout << "scanPmin / stackPdiff = " << mParam.acqStack.mScanPmin / mW << " mW/" << mParam.acqStack.mStackPinc / mWpum << " (mW/um)\n\n";
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

std::string Sequencer::Commandline::actionToString_(const Action::ID action) const
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

#pragma region "Sequencer"
//Constructor using the sample's ROI. The number of stacks is calculated automatically based on the FFOV
Sequencer::Sequencer(const Sample sample, const Stack stack) :
	mSample{ sample },
	mStack{ stack }
{
	initializeVibratomeSlice_();
	initializeTileArrayDimIJ_();	//Calculate the number of tiles in the X-stage and Y-stage axes based on the sample size and the FFOV
	initializeEffectiveROI_();		//Calculate the effective ROI covered by all the tiles, which might be slightly larger than the requested ROI
	reserveMemoryBlock_();			//Reserve memory for speed
}

Sequencer::Commandline Sequencer::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

//Generate a scan pattern and use the vibratome to slice the sample
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";
	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		initializeIteratorIJ_();		//Reset the tile iterator after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages after every cut

		for (std::vector<int>::size_type iterWL = 0; iterWL != mSample.mFluorLabelList.size(); iterWL++)
		{
			//The Y-stage is the slowest to react because it sits under the 2 other stages. For the best performance, iterate over II often and over JJ less often
			while (mJJ >= 0 && mJJ < mTileArraySizeIJ.JJ)		//Y-stage direction. JJ iterates from 0 to mTileArraySizeIJ.JJ
			{
				while (mII >= 0 && mII < mTileArraySizeIJ.II)	//X-stage direction. II iterates back and forth between 0 and mTileArraySizeIJ.II
				{
					moveStage_({ mII, mJJ });
					acqStack_(iterWL);
					saveStack_();
					mII -= SCANDIRtoInt(mIterScanDirXYZ.XX);	//Increase/decrease the iterator in the X-stage axis
				}
				mII += SCANDIRtoInt(mIterScanDirXYZ.XX);		//Re-initialize II by going back one step to start from 0 or mTileArraySizeIJ.II - 1
				reverseSCANDIR(mIterScanDirXYZ.XX);				//Reverse the scanning direction
				mJJ -= SCANDIRtoInt(mIterScanDirXYZ.YY);		//Increase/decrease the iterator in the Y-stage axis
			}
			mJJ += SCANDIRtoInt(mIterScanDirXYZ.YY);			//Re-initialize JJ by going back one step to start from 0 or mTileArraySizeIJ.JJ - 1
			reverseSCANDIR(mIterScanDirXYZ.YY);					//Reverse the scanning direction
		}
		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

/*
//Like generateCommandList() but instead of a list of command, Generate a list of locations to be called by frameByFrameZscanTilingXY()
//Note that this sequence does not use the vibratome
std::vector<POSITION2> Sequencer::generateLocationList()
{
	std::vector<POSITION2> locationList;
	//std::cout << "Generating the location list..." << "\n";

	int II{ 0 }, JJ{ 0 };			//Reset the tile indices after every cut
	resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

	//The stage Y is the slowest to react because it sits under the 2 other stages. For the best performance, iterate over II often and over JJ less often
	while (JJ >= 0 && JJ < mTileArraySizeIJ.JJ)			//Y-stage direction
	{
		while (II >= 0 && II < mTileArraySizeIJ.II)		//X-stage direction
		{
			const POSITION2 tileCenterXY{ indicesToTileCenter_({ II, JJ }) };
			locationList.push_back(tileCenterXY);

			std::cout << "x = " << tileCenterXY.XX / mm << "\ty = " << tileCenterXY.YY / mm << "\n";		//For debugging
			II += SCANDIRtoInt(mIterScanDirXYZ.XX);	//Increase/decrease the iterator in the X-stage axis
		}
		II -= SCANDIRtoInt(mIterScanDirXYZ.XX);		//Re-initialize II by going back one step to start from 0 or mTileArraySizeIJ.II - 1
		reverseSCANDIR(mIterScanDirXYZ.XX);			//Reverse the scanning direction
		JJ += SCANDIRtoInt(mIterScanDirXYZ.YY);		//Increase/decrease the iterator in the Y-stage axis
	}
	JJ -= SCANDIRtoInt(mIterScanDirXYZ.YY);			//Re-initialize JJ by going back one step to start from 0 or mTileArraySizeIJ.JJ - 1
	reverseSCANDIR(mIterScanDirXYZ.YY);				//Reverse the scanning direction

	return locationList;
}
*/

int Sequencer::size() const
{
	return mCommandCounter;
}

Stack Sequencer::stack() const
{
	return mStack;
}

void Sequencer::printSequenceParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan direction {stageX, stageY, stageZ} = {" << SCANDIRtoInt(mInitialScanDirXYZ.XX) << ", " << SCANDIRtoInt(mInitialScanDirXYZ.YY) << ", " << SCANDIRtoInt(mInitialScanDirXYZ.ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Effective ROI (stage positions) [YMIN, XMIN, YMAX, XMAX] = [" << mROIeff.YMIN / mm << " mm, " << mROIeff.XMIN / mm << " mm, " << mROIeff.YMAX / mm << " mm, " << mROIeff.XMAX / mm << " mm]\n";
	*fileHandle << "Effective sample size (stageX, stageY, stageZ) = (" << effectiveSizeXYZ_().XX / mm << " mm, " << effectiveSizeXYZ_().YY / mm << " mm, " << effectiveSizeXYZ_().ZZ / mm << " mm)\n";
	*fileHandle << "Z position of the surface of the sample = " << mSample.mSurfaceZ / mm << " mm\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "Total # tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "Tile array size (stageX, stageY) = (" << mTileArraySizeIJ.II << ", " << mTileArraySizeIJ.JJ << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";

	const double imagingTimePerStack{ g_lineclockHalfPeriod * (560. / 35) * (mStack.mDepth / mStack.mStepSizeZ) };
	const double totalImagingTime_hours{ mStackCounter * imagingTimePerStack / seconds / 3600.};
	*fileHandle << "Runtime per stack = " << imagingTimePerStack / ms << " ms (pixelwidth manually input)\n";
	*fileHandle << "Estimated total runtime (multibeam + pipelining) = " << totalImagingTime_hours << " hrs (pixelwidth manually input)\n\n";
}

//Print the commandlist to file
void Sequencer::printToFile(const std::string fileName) const
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
		//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mTileArraySizeIJ.at(X) * mTileArraySizeIJ.at(Y)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}

void Sequencer::initializeVibratomeSlice_()
{
	mScanZi = mSample.mSurfaceZ; 	//Initialize the Z-stage with the position of the surface of the sample

	mPlaneToSliceZ = mScanZi + mStack.mDepth - mSample.mCutAboveBottomOfStack;

	const int nZZ{ static_cast<int>(1 + std::ceil(1. / (1 - mStack.mOverlap_frac.ZZ) * (mSample.mSizeRequest.ZZ / mStack.mDepth - 1))) };
	if (nZZ > 1)
		mNtotalSlices = nZZ;		//Total number of vibratome slices in the entire sample
	else
		mNtotalSlices = 1;
}

//Calculate the number of tiles in the X-stage and Y-stage axes based on the sample size and the FFOV
//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
void Sequencer::initializeTileArrayDimIJ_()
{
	const int nXX{ static_cast<int>(std::ceil(1 + 1. / (1 - mStack.mOverlap_frac.XX) * (mSample.mSizeRequest.XX / mStack.mFFOV.XX - 1))) };
	const int nYY{ static_cast<int>(std::ceil(1 + 1. / (1 - mStack.mOverlap_frac.YY) * (mSample.mSizeRequest.YY / mStack.mFFOV.YY - 1))) };

	if (nXX > 1)
		mTileArraySizeIJ.II = nXX;		//Number of tiles in the X-stage axis
	else
		mTileArraySizeIJ.II = 1;

	if (nYY > 1)
		mTileArraySizeIJ.JJ = nYY;		//Number of tiles in the Y-stage axis	
	else
		mTileArraySizeIJ.JJ = 1;
}

//Calculate the effective ROI =  { YMIN, XMIN, YMAX, XMAX } covered by all the tiles
void Sequencer::initializeEffectiveROI_()
{
	const POSITION2 tileCenterMin = indicesToTileCenter_({ 0, 0 });
	const POSITION2 tileCenterMax = indicesToTileCenter_({ mTileArraySizeIJ.II - 1, mTileArraySizeIJ.JJ - 1 });
	mROIeff.XMAX = tileCenterMin.XX + mStack.mFFOV.XX / 2;
	mROIeff.YMAX = tileCenterMin.YY + mStack.mFFOV.YY / 2;
	mROIeff.XMIN = tileCenterMax.XX - mStack.mFFOV.XX / 2;
	mROIeff.YMIN = tileCenterMax.YY - mStack.mFFOV.YY / 2;
}

//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
void Sequencer::reserveMemoryBlock_()
{
	const int nTotalTilesPerVibratomeSlice{ mTileArraySizeIJ.II * mTileArraySizeIJ.JJ };													//Total number of tiles in a vibratome slice
	const int nTotalTilesEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * nTotalTilesPerVibratomeSlice };	//Total number of tiles in the entire sample. mNtotalSlices is fixed at 1
	mCommandList.reserve(3 * nTotalTilesEntireSample + mNtotalSlices - 1);
}

//II is the row index of the tile wrt the stitched image, which coincide with the X-stage direction
//JJ is the column index of the tile wrt the stitched image, which coincide with the Y-stage direction
POSITION2 Sequencer::indicesToTileCenter_(const INDICES2 tileIJ) const
{
	if (tileIJ.II < 0 || tileIJ.II >= mTileArraySizeIJ.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + toString(mTileArraySizeIJ.II, 0) + "]");

	if (tileIJ.JJ < 0 || tileIJ.JJ >= mTileArraySizeIJ.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + toString(mTileArraySizeIJ.JJ, 0) + "]");

	//The first tile center is FFOV/2 away from the ROI's edge. The next center is at (1-a)*FFOV away from the first center, where a*L is the tile overlap
	POSITION2 stagePositionXY;
	stagePositionXY.XX = mSample.mCenterXY.XX - mStack.mFFOV.XX  * ((1 - mStack.mOverlap_frac.XX) * (tileIJ.II - (mTileArraySizeIJ.II - 1) / 2));
	stagePositionXY.YY = mSample.mCenterXY.YY - mStack.mFFOV.YY  * ((1 - mStack.mOverlap_frac.YY) * (tileIJ.JJ - (mTileArraySizeIJ.JJ - 1) / 2));

	return stagePositionXY;
}

//Reset the iterators mII and mJJ to the initial values
void Sequencer::initializeIteratorIJ_()
{
	switch (mIterScanDirXYZ.XX)
	{
	case SCANDIR::RIGHTWARD:	//The X-stage moves to the right, therefore, the sample is imaged from right to left
		mII = mTileArraySizeIJ.II - 1;
		break;
	case SCANDIR::LEFTWARD:		//The X-stage moves to the left, therefore, the sample is imaged from left to right
		mII = 0;
		break;
	}
	switch (mIterScanDirXYZ.YY)
	{
	case SCANDIR::INWARD:		//The Y-stage moves inward, therefore, the sample is imaged from "inside" to "outside" of the microscope
		mJJ = mTileArraySizeIJ.JJ - 1;
		break;
	case SCANDIR::OUTWARD:		//The Y-stage moves outward, therefore, the sample is imaged from "outside" to "inside" of the microscope
		mJJ = 0;
		break;
	}
}

void Sequencer::resetStageScanDirections_()
{
	mIterScanDirXYZ = mInitialScanDirXYZ;
}

//Convert a ROI = {ymin, xmin, ymax, xmax} to the equivalent sample size in the X-stage and Y-stage axes
SAMPLESIZE3 Sequencer::effectiveSizeXYZ_() const
{
	return { mROIeff.XMAX - mROIeff.XMIN, mROIeff.YMAX - mROIeff.YMIN, mStack.mDepth * ((1 - mStack.mOverlap_frac.ZZ) * (mNtotalSlices - 1) + 1) };
}

//Move the stage to the position corresponding to the tile indices II and JJ 
void Sequencer::moveStage_(const INDICES2 tilesIJ)
{
	if (tilesIJ.II < 0 || tilesIJ.II >= mTileArraySizeIJ.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + toString(mTileArraySizeIJ.II, 0) + "]");

	if (tilesIJ.JJ < 0 || tilesIJ.JJ >= mTileArraySizeIJ.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + toString(mTileArraySizeIJ.JJ, 0) + "]");

	const POSITION2 tileCenterXY = indicesToTileCenter_(tilesIJ);

	Commandline commandline{ Action::ID::MOV };
	commandline.mParam.moveStage = { mSliceCounter, tilesIJ, tileCenterXY };
	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int wavelengthIndex)
{
	if (wavelengthIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength index must be >=0");


	//Read the corresponding laser configuration
	const FluorLabelList::FluorLabel fluorLabel{ mSample.mFluorLabelList.at(wavelengthIndex) };

	Commandline commandline{ Action::ID::ACQ };
	commandline.mParam.acqStack = { mStackCounter, fluorLabel.mWavelength_nm, mIterScanDirXYZ.ZZ, mScanZi, mStack.mDepth, fluorLabel.mScanPmin, fluorLabel.mStackPinc, fluorLabel.nFramesBinning };
	mCommandList.push_back(commandline);
	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	reverseSCANDIR(mIterScanDirXYZ.ZZ);								//Reverse the scanning direction in the Z-stage axis
}

void Sequencer::saveStack_()
{
	Commandline commandline{ Action::ID::SAV };
	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequencer::cutSlice_()
{
	//Move the sample to face the vibratome blade. Note the additional offset in the Z-stage axis
	const POSITION3 samplePositionXYZ{ mSample.mBladePositionXY.XX, mSample.mBladePositionXY.YY, mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline{ Action::ID::CUT };
	commandline.mParam.cutSlice = { samplePositionXYZ };
	mCommandList.push_back(commandline);
	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the Z-stage and the height of the plane to be cut in the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlap_frac.ZZ) * mStack.mDepth;
	mPlaneToSliceZ += (1 - mStack.mOverlap_frac.ZZ) * mStack.mDepth;
}
#pragma endregion "sequencer"