#include "Sequencer.h"

#pragma region "Commandline"
std::string Commandline::actionToString_(const ACTION action) const
{
	switch (action)
	{
	case ACTION::CUT:
		return "CUT";
	case ACTION::SAV:
		return "SAV";
	case ACTION::ACQ:
		return "ACQ";
	case ACTION::MOV:
		return "MOV";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

std::string Commandline::printHeader() const
{
	return 	"Action\tSlice#\tStackIJ\tStack Center\tStack#\tWavlen\tDirZ\tZi\tZf\tPi\tPf";
}

std::string Commandline::printHeaderUnits() const
{
	return "\t\t(mm,mm)\t\t\tnm\t\tmm\tmm\tmW\tmW";
}

void Commandline::printToFile(std::ofstream *fileHandle) const
{
	double scanZf, scanPf;

	switch (mAction)
	{
	case ACTION::MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mCommand.moveStage.mSliceNumber;
		*fileHandle << "\t(" << mCommand.moveStage.mStackIJ.at(STAGEX) << "," << mCommand.moveStage.mStackIJ.at(STAGEY) << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mCommand.moveStage.mStackCenterXY.at(STAGEX) / mm << "," << mCommand.moveStage.mStackCenterXY.at(STAGEY) / mm << ")\n";
		break;
	case ACTION::ACQ:
		scanZf = mCommand.acqStack.mScanZi + mCommand.acqStack.mScanDirZ * mCommand.acqStack.mStackDepth;
		scanPf = mCommand.acqStack.mScanPi + mCommand.acqStack.mScanDirZ * mCommand.acqStack.mStackPinc;
		
		*fileHandle << actionToString_(mAction) << "\t\t\t\t\t";
		*fileHandle << mCommand.acqStack.mStackNumber << "\t";
		*fileHandle << mCommand.acqStack.mWavelength_nm << "\t";
		*fileHandle << mCommand.acqStack.mScanDirZ << "\t";
		*fileHandle << std::setprecision(3);
		*fileHandle << mCommand.acqStack.mScanZi / mm << "\t";
		*fileHandle << scanZf / mm << "\t";
		*fileHandle << std::setprecision(0);
		*fileHandle << mCommand.acqStack.mScanPi << "\t" << scanPf << "\n";
		break;
	case ACTION::SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case ACTION::CUT:
		*fileHandle << actionToString_(mAction);
		*fileHandle << std::setprecision(3);
		*fileHandle << "\t************Sample facing the vibratome at (mm) = ";
		*fileHandle << "(" << mCommand.cutSlice.mBladePositionXY.at(STAGEX) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(STAGEY) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(STAGEZ) / mm << ")";
		*fileHandle << "**************************************************************\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

void Commandline::printParameters() const
{
	switch (mAction)
	{
	case ACTION::MOV:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mCommand.moveStage.mSliceNumber << "\n";
		std::cout << "Stack ij = (" << mCommand.moveStage.mStackIJ.at(STAGEX) << "," << mCommand.moveStage.mStackIJ.at(STAGEY) << ")\n";
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.mStackCenterXY.at(STAGEX) / mm<< "," << mCommand.moveStage.mStackCenterXY.at(STAGEY) / mm << ")\n\n";
		break;
	case ACTION::ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength (nm) = " << mCommand.acqStack.mWavelength_nm << "\n";
		std::cout << "scanDirZ = " << mCommand.acqStack.mScanDirZ << "\n";
		std::cout << "scanZi (mm) / stackDepth (mm) = " << mCommand.acqStack.mScanZi / mm << "/" << mCommand.acqStack.mStackDepth << "\n";
		std::cout << "scanPi (mW) / stackPdiff (mW/um) = " << mCommand.acqStack.mScanPi / mW << "/" << mCommand.acqStack.mStackPinc / mWpum << "\n\n";
		break;
	case ACTION::SAV:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	case ACTION::CUT:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequencer"
//Constructor using the sample's ROI. The number of stacks is calculated automatically based on the FFOV
Sequencer::Sequencer(const ChannelList channelList, const Sample sample, const Stack stack) : mSample{ sample }, mChannelList{ channelList }, mStack{ stack }
{
	//Initialize the z-stage with the position of the sample surface
	mScanZi = mSample.mSurfaceZ;

	//Initialize the height of the plane to slice
	mPlaneToSliceZ = mScanZi + mStack.mDepth - mSample.mCutAboveBottomOfStack;

	//Calculate the number of stacks in X and Y based on the sample size and stack FFOV
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(STAGEX) };		//Dummy local variable
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(STAGEY) };		//Dummy local variable
	mStackArrayDimIJ.at(STAGEX) = static_cast<int>(std::ceil(  1/(1-overlapX_frac) * (mSample.mLengthXYZ.at(STAGEX) / mStack.mFFOV.at(STAGEX) - 1) + 1  ));		//Number of stacks in x
	mStackArrayDimIJ.at(STAGEY) = static_cast<int>(std::ceil(  1/(1-overlapY_frac) * (mSample.mLengthXYZ.at(STAGEY) / mStack.mFFOV.at(STAGEY) - 1) + 1  ));		//Number of stacks in y

	//Calculate the total number of stacks in a vibratome slice and also in the entire sample
	const double overlapZ_frac{ mStack.mOverlapXYZ_frac.at(STAGEZ) };																		//Dummy local variable
	mNtotalSlices = static_cast<int>(std::ceil(  1 / (1 - overlapZ_frac) * (mSample.mLengthXYZ.at(STAGEZ) / mStack.mDepth - 1) + 1  ));		//Total number of slices in the entire sample
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(STAGEX) * mStackArrayDimIJ.at(STAGEY) };									//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mChannelList.size()) * mNtotalStacksPerVibratomeSlice };			//Total number of stacks in the entire sample

	//Calculate the ROI effectively covered by the stacks, which might be slightly larger than the sample's ROI
	mROIcovered.at(XMIN) = mSample.mROI.at(XMIN);
	mROIcovered.at(YMIN) = mSample.mROI.at(YMIN);
	mROIcovered.at(XMAX) = mSample.mROI.at(XMIN) + mStack.mFFOV.at(STAGEX)  * ((1 - overlapX_frac) * mStackArrayDimIJ.at(STAGEX) + 0.5);
	mROIcovered.at(YMAX) = mSample.mROI.at(YMIN) + mStack.mFFOV.at(STAGEY)  * ((1 - overlapY_frac) * mStackArrayDimIJ.at(STAGEY) + 0.5);

	//Pre-reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Check that cutAboveBottomOfStack is greater than the stack z-overlap
	const double stackOverlapZ{ mStack.mOverlapXYZ_frac.at(STAGEZ) * mStack.mDepth };	//Dummy local variable
	if (mSample.mCutAboveBottomOfStack < stackOverlapZ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'cutAboveBottomOfStack' must be greater than the stack z-overlap " + toString(stackOverlapZ / um, 1) + " um");
}

//Constructor using the initial stack center and the number of stacks. To be used without slicing
Sequencer::Sequencer(const ChannelList channelList, Sample sample, const Stack stack, const double3 stackCenterXYZ, const int2 stackArrayDimIJ) :
	mSample{ sample }, mChannelList{ channelList }, mStack{ stack }, mStackArrayDimIJ{ stackArrayDimIJ }
{
	//Calculate the ROI covered by the stacks
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 )
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(STAGEX) };		//Dummy local variable
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(STAGEY) };		//Dummy local variable
	mROIcovered.at(XMIN) = stackCenterXYZ.at(STAGEX) - mStack.mFFOV.at(STAGEX) / 2;
	mROIcovered.at(YMIN) = stackCenterXYZ.at(STAGEY) - mStack.mFFOV.at(STAGEY) / 2;
	mROIcovered.at(XMAX) = mROIcovered.at(XMIN) + mStack.mFFOV.at(STAGEX) * ((1 - overlapX_frac) * (mStackArrayDimIJ.at(STAGEX) - 1) + 1);
	mROIcovered.at(YMAX) = mROIcovered.at(YMIN) + mStack.mFFOV.at(STAGEY) * ((1 - overlapY_frac) * (mStackArrayDimIJ.at(STAGEY) - 1) + 1);

	//Initialize the sample's ROI. Just copy the effective ROI covered
	mSample.mROI = mROIcovered;

	//Initialize the z-stage
	mScanZi = stackCenterXYZ.at(STAGEZ);

	//Initialize the height of the plane to slice
	mPlaneToSliceZ = 0;

	if (stackArrayDimIJ.at(STAGEX) <= 0 || stackArrayDimIJ.at(STAGEY) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack array dimension must be equal to 1 or greater");

	mNtotalSlices = 1;
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(STAGEX) * mStackArrayDimIJ.at(STAGEY) };										//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mChannelList.size()) * mNtotalStacksPerVibratomeSlice };		//Total number of stacks in the entire sample

	//Pre-reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);
}

//The initial laser power of a stack-scan depends on whether the stack is imaged from the top down or from the bottom up.
//Give the laser power at the top of the stack scanPmin, the power increment stackPinc, and the scanning direction scanDirZ to return the initial laser power of the scan
double Sequencer::calculateStackScanInitialPower_(const double scanPmin, const double stackPinc, const int scanDirZ)
{
	if (scanDirZ > 0)	//z-stage scans upwards
		return scanPmin;
	else				//z-stage scans downwards
		return scanPmin + stackPinc;
}

//The first stack center is L/2 away from the ROI's edge. The next center is at (1-a)*L away from the first center, where a*L is the stack overlap
double2 Sequencer::stackIndicesToStackCenter_(const int2 stackArrayIndicesIJ) const
{
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(STAGEX) };
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(STAGEY) };

	double2 stagePositionXY;
	stagePositionXY.at(STAGEX) = mSample.mROI.at(XMIN) + mStack.mFFOV.at(STAGEX)  * ((1 - overlapX_frac) * stackArrayIndicesIJ.at(STAGEX) + 0.5);
	stagePositionXY.at(STAGEY) = mSample.mROI.at(YMIN) + mStack.mFFOV.at(STAGEY)  * ((1 - overlapY_frac) * stackArrayIndicesIJ.at(STAGEY) + 0.5);

	return stagePositionXY;
}

void Sequencer::reverseStageScanDirection_(const Axis axis)
{
	switch (axis)
	{
	case STAGEX:
		mScanDir.at(STAGEX) *= -1;
		break;
	case STAGEY:
		mScanDir.at(STAGEY) *= -1;
		break;
	case STAGEZ:
		mScanDir.at(STAGEZ) *= -1;
		break;
	}
}

void Sequencer::resetStageScanDirections_()
{
	mScanDir = mInitialScanDir;
}

void Sequencer::moveStage_(const int2 stackIJ)
{
	const double2 stackCenterXY = stackIndicesToStackCenter_(stackIJ);

	Commandline commandline;
	commandline.mAction = ACTION::MOV;
	commandline.mCommand.moveStage = { mSliceCounter, stackIJ, stackCenterXY };
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int iterWL)
{
	//Read the corresponding laser configuration
	const ChannelList::SingleChannel singleLaser{ mChannelList.at(iterWL) };

	//Determine if the initial laser power is the lowest (top of the stack) or the highest (bottom of the stack)
	const double scanPi{ calculateStackScanInitialPower_(singleLaser.mScanPi, singleLaser.mStackPinc, mScanDir.at(STAGEZ)) };

	Commandline commandline;
	commandline.mAction = ACTION::ACQ;
	commandline.mCommand.acqStack = { mStackCounter, singleLaser.mWavelength_nm, mScanDir.at(STAGEZ), mScanZi, mStack.mDepth, scanPi, singleLaser.mStackPinc };
	mCommandList.push_back(commandline);

	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	mScanZi += mScanDir.at(STAGEZ) * mStack.mDepth;		//Next initial z-scan position
	reverseStageScanDirection_(STAGEZ);					//Switch the scanning direction in z
}

void Sequencer::saveStack_()
{
	Commandline commandline;
	commandline.mAction = ACTION::SAV;
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::cutSlice_()
{
	//Move the sample to face the vibratome blade. Notice the additional offset in z
	const double3 samplePositionXYZ{ mSample.mBladePositionXY.at(STAGEX), mSample.mBladePositionXY.at(STAGEY), mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline;
	commandline.mAction = ACTION::CUT;
	commandline.mCommand.cutSlice = { samplePositionXYZ };
	mCommandList.push_back(commandline);

	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the z-stage for the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlapXYZ_frac.at(STAGEZ)) * mStack.mDepth;

	//Increase the height of the plane to be cut for the next iteration
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mPlaneToSliceZ += (1 - mStack.mOverlapXYZ_frac.at(STAGEZ)) * mStack.mDepth;
}

//To generate a scan pattern with slicing
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";

	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

		for (std::vector<int>::size_type iterWL = 0; iterWL != mChannelList.size(); iterWL++)
		{
			//The y-stage is the slowest to react because it sits under of other 2 stages. For the best performance, iterate over x often and over y less often
			while (JJ >= 0 && JJ < mStackArrayDimIJ.at(STAGEY))			//y direction
			{
				while (II >= 0 && II < mStackArrayDimIJ.at(STAGEX))		//x direction
				{
					moveStage_({ II, JJ });
					acqStack_(iterWL);
					saveStack_();
					II += mScanDir.at(STAGEX);		//Increase the iterator x
				}

				//Initialize the next cycle by going back in x one step and switching the scanning direction
				II -= mScanDir.at(STAGEX);
				reverseStageScanDirection_(STAGEX);
				JJ += mScanDir.at(STAGEY);	//Increase the iterator y
			}
			//Initialize the next cycle by going back in y one step and switching the scanning direction
			JJ -= mScanDir.at(STAGEY);
			reverseStageScanDirection_(STAGEY);
		}

		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

//To generate a scan pattern without slicing
std::vector<double2> Sequencer::generateLocationList()
{
	std::vector<double2> locationList;
	//std::cout << "Generating the location list..." << "\n";

	int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
	resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

	//The y-stage is the slowest to react because it sits under of other 2 stages. For the best performance, iterate over x often and over y less often
	while (JJ >= 0 && JJ < mStackArrayDimIJ.at(STAGEY))			//y direction
	{
		while (II >= 0 && II < mStackArrayDimIJ.at(STAGEX))		//x direction
		{
			const double2 stackCenterXY = stackIndicesToStackCenter_({ II, JJ });
			locationList.push_back(stackCenterXY);
			
			//std::cout << "x = " << stackCenterXY.at(STAGEX) / mm << "\ty = " << stackCenterXY.at(STAGEY) / mm << "\n";		//For debugging
			II += mScanDir.at(STAGEX);		//Increase the iterator x
		}

		//Initialize the next cycle by going back in x one step and switching the scanning direction
		II -= mScanDir.at(STAGEX);
		reverseStageScanDirection_(STAGEX);
		JJ += mScanDir.at(STAGEY);	//Increase the iterator y
	}
	//Initialize the next cycle by going back in y one step and switching the scanning direction
	JJ -= mScanDir.at(STAGEY);
	reverseStageScanDirection_(STAGEY);

	return locationList;
}


Commandline Sequencer::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

void Sequencer::printSequencerParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan directions (x,y,z) = {" << mInitialScanDir.at(STAGEX) << "," << mInitialScanDir.at(STAGEY) << "," << mInitialScanDir.at(STAGEZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "ROI covered [YMIN, XMIN, YMAX, XMAX] (mm) = [" << mROIcovered.at(YMIN) / mm << "," << mROIcovered.at(XMIN) / mm << "," << mROIcovered.at(YMAX) / mm << "," << mROIcovered.at(XMAX) / mm << "]\n";
	*fileHandle << "Sample surface z position (mm) = " << mSample.mSurfaceZ / mm << "\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "# tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (x,y) = (" << mStackArrayDimIJ.at(STAGEX) << "," << mStackArrayDimIJ.at(STAGEY) << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";
	*fileHandle << "\n";
}

//Print the commandlist to file
void Sequencer::printToFile(const std::string fileName) const
{
	std::ofstream *fileHandle{ new std::ofstream(folderPath + fileName + ".txt") };

	*fileHandle << std::fixed;	//Show a fixed number of digits

	mSample.printParams(fileHandle);
	mChannelList.printParams(fileHandle);
	printSequencerParams(fileHandle);
	mStack.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
	//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mStackArrayDimIJ.at(STAGEX) * mStackArrayDimIJ.at(STAGEY)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}
#pragma endregion "sequencer"