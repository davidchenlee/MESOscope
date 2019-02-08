#include "Sequencer.h"

#pragma region "Commandline"
std::string Commandline::actionToString_(const Action action) const
{
	switch (action)
	{
	case CUT:
		return "CUT";
	case SAV:
		return "SAV";
	case ACQ:
		return "ACQ";
	case MOV:
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
	case MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mCommand.moveStage.mSliceNumber;
		*fileHandle << "\t(" << mCommand.moveStage.mStackIJ.at(XX) << "," << mCommand.moveStage.mStackIJ.at(YY) << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mCommand.moveStage.mStackCenterXY.at(XX) / mm << "," << mCommand.moveStage.mStackCenterXY.at(YY) / mm << ")\n";
		break;
	case ACQ:
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
	case SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case CUT:
		*fileHandle << actionToString_(mAction);
		*fileHandle << std::setprecision(3);
		*fileHandle << "\t************Sample facing the vibratome at (mm) = ";
		*fileHandle << "(" << mCommand.cutSlice.mBladePositionXY.at(XX) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(YY) / mm << "," << mCommand.cutSlice.mBladePositionXY.at(ZZ) / mm << ")";
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
	case MOV:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mCommand.moveStage.mSliceNumber << "\n";
		std::cout << "Stack ij = (" << mCommand.moveStage.mStackIJ.at(XX) << "," << mCommand.moveStage.mStackIJ.at(YY) << ")\n";
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.mStackCenterXY.at(XX) / mm<< "," << mCommand.moveStage.mStackCenterXY.at(YY) / mm << ")\n\n";
		break;
	case ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength (nm) = " << mCommand.acqStack.mWavelength_nm << "\n";
		std::cout << "scanDirZ = " << mCommand.acqStack.mScanDirZ << "\n";
		std::cout << "scanZi/stackDepth (mm) = " << mCommand.acqStack.mScanZi / mm << "/" << mCommand.acqStack.mStackDepth << "\n";
		std::cout << "scanPi/stackPdiff (mW) = " << mCommand.acqStack.mScanPi / mW << "/" << mCommand.acqStack.mStackPinc / mW << "\n\n";
		break;
	case SAV:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	case CUT:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequencer"
//Constructor using the sample's ROI. The number of stacks is calculated automatically based on the FFOV
Sequencer::Sequencer(const LaserList laserList, const Sample sample, const Stack stack) : mSample(sample), mLaserList(laserList), mStack(stack)
{
	//Initialize the z-stage with the position of the sample surface
	mScanZi = mSample.mSurfaceZ;

	//Initialize the height of the plane to slice
	mPlaneToSliceZ = mScanZi + mStack.mDepth - mSample.mCutAboveBottomOfStack;

	//Calculate the number of stacks in X and Y based on the sample size and stack FFOV
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(XX) };		//Dummy local variable
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(YY) };		//Dummy local variable
	mStackArrayDimIJ.at(XX) = static_cast<int>(std::ceil(  1/(1-overlapX_frac) * (mSample.mLengthXYZ.at(XX) / mStack.mFFOV.at(XX) - 1) + 1  ));		//Number of stacks in x
	mStackArrayDimIJ.at(YY) = static_cast<int>(std::ceil(  1/(1-overlapY_frac) * (mSample.mLengthXYZ.at(YY) / mStack.mFFOV.at(YY) - 1) + 1  ));		//Number of stacks in y

	//Calculate the total number of stacks in a vibratome slice and also in the entire sample
	const double overlapZ_frac{ mStack.mOverlapXYZ_frac.at(ZZ) };																		//Dummy local variable
	mNtotalSlices = static_cast<int>(std::ceil(  1 / (1 - overlapZ_frac) * (mSample.mLengthXYZ.at(ZZ) / mStack.mDepth - 1) + 1  ));		//Total number of slices in the entire sample
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(XX) * mStackArrayDimIJ.at(YY) };										//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mLaserList.listSize()) * mNtotalStacksPerVibratomeSlice };		//Total number of stacks in the entire sample

	//Calculate the ROI effectively covered by the stacks, which might be slightly larger than the sample's ROI
	mROIcovered.at(XMIN) = mSample.mROI.at(XMIN);
	mROIcovered.at(YMIN) = mSample.mROI.at(YMIN);
	mROIcovered.at(XMAX) = mSample.mROI.at(XMIN) + mStack.mFFOV.at(XX)  * ((1 - overlapX_frac) * mStackArrayDimIJ.at(XX) + 0.5);
	mROIcovered.at(YMAX) = mSample.mROI.at(YMIN) + mStack.mFFOV.at(YY)  * ((1 - overlapY_frac) * mStackArrayDimIJ.at(YY) + 0.5);

	//Pre-reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	//Check that cutAboveBottomOfStack is greater than the stack z-overlap
	const double stackOverlapZ{ mStack.mOverlapXYZ_frac.at(ZZ) * mStack.mDepth };	//Dummy local variable
	if (mSample.mCutAboveBottomOfStack < stackOverlapZ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'cutAboveBottomOfStack' must be greater than the stack z-overlap " + toString(stackOverlapZ / um, 1) + " um");
}

//Constructor using the initial stack center and number of stacks. To be used with the bead slide and therefore no slicing
Sequencer::Sequencer(const LaserList laserList, Sample sample, const Stack stack, const double3 stackCenterXYZ, const int2 stackArrayDimIJ) : mSample(sample), mLaserList(laserList), mStack(stack), mStackArrayDimIJ(stackArrayDimIJ)
{
	//Calculate the ROI covered by the stacks
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 )
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(XX) };		//Dummy local variable
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(YY) };		//Dummy local variable
	mROIcovered.at(XMIN) = stackCenterXYZ.at(XX) - mStack.mFFOV.at(XX) / 2;
	mROIcovered.at(YMIN) = stackCenterXYZ.at(YY) - mStack.mFFOV.at(YY) / 2;
	mROIcovered.at(XMAX) = mROIcovered.at(XMIN) + mStack.mFFOV.at(XX) * ((1 - overlapX_frac) * (mStackArrayDimIJ.at(XX) - 1) + 1);
	mROIcovered.at(YMAX) = mROIcovered.at(YMIN) + mStack.mFFOV.at(YY) * ((1 - overlapY_frac) * (mStackArrayDimIJ.at(YY) - 1) + 1);

	//Initialize the sample's ROI. Just copy the effective ROI covered
	mSample.mROI = mROIcovered;

	//Initialize the z-stage
	mScanZi = stackCenterXYZ.at(ZZ) - mStack.mDepth / 2;

	//Initialize the height of the plane to slice
	mPlaneToSliceZ = 0;

	if (stackArrayDimIJ.at(XX) <= 0 || stackArrayDimIJ.at(YY) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack array dimension must be equal to 1 or greater");

	mNtotalSlices = 1;
	const int mNtotalStacksPerVibratomeSlice{ mStackArrayDimIJ.at(XX) * mStackArrayDimIJ.at(YY) };										//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample{ mNtotalSlices * static_cast<int>(mLaserList.listSize()) * mNtotalStacksPerVibratomeSlice };		//Total number of stacks in the entire sample

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
	const double overlapX_frac{ mStack.mOverlapXYZ_frac.at(XX) };
	const double overlapY_frac{ mStack.mOverlapXYZ_frac.at(YY) };

	double2 stagePositionXY;
	stagePositionXY.at(XX) = mSample.mROI.at(XMIN) + mStack.mFFOV.at(XX)  * ((1 - overlapX_frac) * stackArrayIndicesIJ.at(XX) + 0.5);
	stagePositionXY.at(YY) = mSample.mROI.at(YMIN) + mStack.mFFOV.at(YY)  * ((1 - overlapY_frac) * stackArrayIndicesIJ.at(YY) + 0.5);

	return stagePositionXY;
}

void Sequencer::reverseStageScanDirection_(const Axis axis)
{
	switch (axis)
	{
	case XX:
		mScanDir.at(XX) *= -1;
		break;
	case YY:
		mScanDir.at(YY) *= -1;
		break;
	case ZZ:
		mScanDir.at(ZZ) *= -1;
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
	commandline.mAction = MOV;
	commandline.mCommand.moveStage = { mSliceCounter, stackIJ, stackCenterXY };
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int iterWL)
{
	//Read the corresponding laser configuration
	const LaserList::SingleLaser singleLaser{ mLaserList.mLaser.at(iterWL) };

	//Determine if the initial laser power is the lowest (top of the stack) or the highest (bottom of the stack)
	const double scanPi{ calculateStackScanInitialPower_(singleLaser.mScanPi, singleLaser.mStackPinc, mScanDir.at(ZZ)) };

	Commandline commandline;
	commandline.mAction = ACQ;
	commandline.mCommand.acqStack = { mStackCounter, singleLaser.mWavelength_nm, mScanDir.at(ZZ), mScanZi, mStack.mDepth, scanPi, singleLaser.mStackPinc };
	mCommandList.push_back(commandline);

	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	mScanZi += mScanDir.at(ZZ) * mStack.mDepth;		//Next initial z-scan position
	reverseStageScanDirection_(ZZ);								//Switch the scanning direction in z
}

void Sequencer::saveStack_()
{
	Commandline commandline;
	commandline.mAction = SAV;
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::cutSlice_()
{
	//Move the sample to face the vibratome blade. Notice the additional offset in z
	const double3 samplePositionXYZ{ mSample.mBladePositionXY.at(XX), mSample.mBladePositionXY.at(YY), mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline;
	commandline.mAction = CUT;
	commandline.mCommand.cutSlice = { samplePositionXYZ };
	mCommandList.push_back(commandline);

	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the z-stage for the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlapXYZ_frac.at(ZZ)) * mStack.mDepth;

	//Increase the height of the plane to be cut for the next iteration
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mPlaneToSliceZ += (1 - mStack.mOverlapXYZ_frac.at(ZZ)) * mStack.mDepth;
}

//To generate a scan pattern with slicing
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";

	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		int II{ 0 }, JJ{ 0 };			//Reset the stack indices after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

		for (std::vector<int>::size_type iterWL = 0; iterWL != mLaserList.listSize(); iterWL++)
		{
			//The y-stage is the slowest to react because it sits under of other 2 stages. For the best performance, iterate over x often and over y less often
			while (JJ >= 0 && JJ < mStackArrayDimIJ.at(YY))			//y direction
			{
				while (II >= 0 && II < mStackArrayDimIJ.at(XX))		//x direction
				{
					moveStage_({ II, JJ });
					acqStack_(iterWL);
					saveStack_();
					II += mScanDir.at(XX);		//Increase the iterator x
				}

				//Initialize the next cycle by going back in x one step and switching the scanning direction
				II -= mScanDir.at(XX);
				reverseStageScanDirection_(XX);
				JJ += mScanDir.at(YY);	//Increase the iterator y
			}
			//Initialize the next cycle by going back in y one step and switching the scanning direction
			JJ -= mScanDir.at(YY);
			reverseStageScanDirection_(YY);
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
	while (JJ >= 0 && JJ < mStackArrayDimIJ.at(YY))			//y direction
	{
		while (II >= 0 && II < mStackArrayDimIJ.at(XX))		//x direction
		{
			const double2 stackCenterXY = stackIndicesToStackCenter_({ II, JJ });
			locationList.push_back(stackCenterXY);
			
			//std::cout << "x = " << stackCenterXY.at(XX) / mm << "\ty = " << stackCenterXY.at(YY) / mm << "\n";		//For debugging
			II += mScanDir.at(XX);		//Increase the iterator x
		}

		//Initialize the next cycle by going back in x one step and switching the scanning direction
		II -= mScanDir.at(XX);
		reverseStageScanDirection_(XX);
		JJ += mScanDir.at(YY);	//Increase the iterator y
	}
	//Initialize the next cycle by going back in y one step and switching the scanning direction
	JJ -= mScanDir.at(YY);
	reverseStageScanDirection_(YY);

	return locationList;
}


Commandline Sequencer::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

void Sequencer::printSequencerParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan directions (x,y,z) = {" << mInitialScanDir.at(XX) << "," << mInitialScanDir.at(YY) << "," << mInitialScanDir.at(ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "ROI covered [YMIN, XMIN, YMAX, XMAX] (mm) = [" << mROIcovered.at(YMIN) / mm << "," << mROIcovered.at(XMIN) / mm << "," << mROIcovered.at(YMAX) / mm << "," << mROIcovered.at(XMAX) / mm << "]\n";
	*fileHandle << "Sample surface z position (mm) = " << mSample.mSurfaceZ / mm << "\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "# tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (x,y) = (" << mStackArrayDimIJ.at(XX) << "," << mStackArrayDimIJ.at(YY) << ")\n";
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
	mLaserList.printParams(fileHandle);
	printSequencerParams(fileHandle);
	mStack.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
	//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mStackArrayDimIJ.at(XX) * mStackArrayDimIJ.at(YY)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}
#pragma endregion "sequencer"