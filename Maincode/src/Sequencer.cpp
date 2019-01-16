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
	return 	"Action\tSlice#\tStackIJ\tPosition\tStack#\tWavlen\tDirZ\tZi\tZf\tPi\tPf";
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
		*fileHandle << std::setprecision(3);
		*fileHandle << "(" << mCommand.moveStage.mStackCenter.at(0) / mm << "," << mCommand.moveStage.mStackCenter.at(1) / mm << ")\n";
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
		*fileHandle << "(" << mCommand.cutSlice.mSamplePosition.at(XX) / mm << "," << mCommand.cutSlice.mSamplePosition.at(YY) / mm << "," << mCommand.cutSlice.mSamplePosition.at(ZZ) / mm << ")";
		*fileHandle << "*******************************\n";
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
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.mStackCenter.at(XX) / mm<< "," << mCommand.moveStage.mStackCenter.at(YY) / mm << ")\n\n";
		break;
	case ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength (nm) = " << mCommand.acqStack.mWavelength_nm << "\n";
		std::cout << "scanDirZ = " << mCommand.acqStack.mScanDirZ << "\n";
		std::cout << "scanZi/stackDepth (mm) = " << mCommand.acqStack.mScanZi / mm << "/" << mCommand.acqStack.mStackDepth << "\n";
		std::cout << "scanPi/stackPdiff (mW) = " << mCommand.acqStack.mScanPi / mW << "/" << mCommand.acqStack.mStackPinc / mW << "\n\n";
		break;
	case SAV:
		std::cout << "The command is " << actionToString_(mAction) << " with no parameters\n";
		break;
	case CUT:
		std::cout << "The command is " << actionToString_(mAction) << " with no parameters\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequencer"
Sequencer::Sequencer(const Sample sample, const LaserList laserList, const Stack stack, const Vibratome vibratome) :
	mSample(sample), mLaserList(laserList), mStack(stack), mVibratome(vibratome)
{
	//Initialize the z-stage
	mScanZi = mSample.mInitialZ;

	//Initialize the height of the plane to sliced
	mPlaneToSliceZ = mScanZi + mStack.mDepth - mVibratome.mCutAboveBottomOfStack;

	//Calculate the total number of stacks in a vibratome slice and also in the entire sample
	//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV ( (1-a)*N + 1 )
	//Therefore, the given L and FOV, then N =1/(1-a) * ( L/FOV - 1 )
	const double overlapX_frac = mStack.mOverlap_frac.at(XX);
	const double overlapY_frac = mStack.mOverlap_frac.at(YY);
	mStackArrayDim.at(XX) = static_cast<int>(std::ceil( 1/(1-overlapX_frac) * (mSample.mLength.at(XX) / mStack.mFOV.at(XX)) - overlapX_frac));		//Number of stacks in x
	mStackArrayDim.at(YY) = static_cast<int>(std::ceil( 1/(1-overlapY_frac) * (mSample.mLength.at(YY) / mStack.mFOV.at(YY)) - overlapY_frac));		//Number of stacks in y

	//Initialize the number of vibratome slices in the entire sample
	const double overlapZ_frac = mStack.mOverlap_frac.at(ZZ);
	mNtotalSlices = static_cast<int>(std::ceil(1 / (1 - overlapZ_frac) * (mSample.mLength.at(ZZ) / mStack.mDepth) - overlapZ_frac));

	const int mNtotalStacksPerVibratomeSlice = mStackArrayDim.at(XX) * mStackArrayDim.at(YY);												//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample = mNtotalSlices * static_cast<int>(mLaserList.listSize()) * mNtotalStacksPerVibratomeSlice;	//Total number of stacks in the entire sample

	//Pre-reserve a memory block assuming 3 actions for every stack in a vibratome slice (MOV, ACQ, and SAV); and then CUT
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);

	const double stackOverlap = mStack.mOverlap_frac.at(ZZ) * mStack.mDepth;
	if (mVibratome.mCutAboveBottomOfStack < stackOverlap)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'cutAboveBottomOfStack' must be greater than the stitching length  " + toString(stackOverlap / um, 1) + " um");
}

//The initial laser power of a stack-scan depends on whether the stack is imaged from the top down or from the bottom up.
//Give the laser power at the top of the stack scanPmin, the power increment stackPinc, and the scanning direction scanDirZ to return the initial laser power of the scan
double Sequencer::calculateStackScanInitialP_(const double scanPmin, const double stackPinc, const int scanDirZ)
{
	if (scanDirZ > 0)	//z-stage scans upwards
		return scanPmin;
	else				//z-stage scans downwards
		return scanPmin + stackPinc;
}

//The first stack center is L/2 away from the ROI's edge. The next center is (1-a)*L away from the first center, where a*L is the stack overlap
double2 Sequencer::stackIndicesToStackCenter_(const int2 stackArrayIndices) const
{
	const double overlapX_frac = mStack.mOverlap_frac.at(XX);
	const double overlapY_frac = mStack.mOverlap_frac.at(YY);

	double2 stagePosition;
	stagePosition.at(XX) = mSample.mROI.at(0) + mStack.mFOV.at(XX)  * ((1 - overlapX_frac) * stackArrayIndices.at(XX) + 0.5);
	stagePosition.at(YY) = mSample.mROI.at(3) + mStack.mFOV.at(YY)  * ((1 - overlapY_frac) * stackArrayIndices.at(YY) + 0.5);

	return stagePosition;
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
	const double2 stackCenter = stackIndicesToStackCenter_(stackIJ);

	Commandline commandline;
	commandline.mAction = MOV;
	commandline.mCommand.moveStage = { mSliceCounter, stackIJ, stackCenter };
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int iterWL)
{
	//Read the corresponding laser configuration
	const LaserList::SingleLaser singleLaser = mLaserList.mLaser.at(iterWL);

	//Determine if the initial laser power is the lowest (top of the stack) or the highest (bottom of the stack)
	const double scanPi = calculateStackScanInitialP_(singleLaser.mScanPi, singleLaser.mStackPinc, mScanDir.at(ZZ));

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
	const double3 samplePosition = { mVibratome.mSamplePosition.at(XX), mVibratome.mSamplePosition.at(YY), mPlaneToSliceZ + mVibratome.mBladeFocalplaneOffsetZ };

	Commandline commandline;
	commandline.mAction = CUT;
	commandline.mCommand.cutSlice = { samplePosition };
	mCommandList.push_back(commandline);

	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the z-stage for the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlap_frac.at(ZZ)) * mStack.mDepth;

	//Increase the height of the plane to be cut for the next iteration
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mPlaneToSliceZ += (1 - mStack.mOverlap_frac.at(ZZ)) * mStack.mDepth;
}

//Snake scanning
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";

	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		int xx = 0, yy = 0;				//Reset the stack indices after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages

		for (std::vector<int>::size_type iterWL = 0; iterWL != mLaserList.listSize(); iterWL++)
		{
			//The y-stage is the slowest stage to react because it sits under of other 2 stages. For the best performance, iterate over y fewer times
			while (yy >= 0 && yy < mStackArrayDim.at(YY))			//y direction
			{
				while (xx >= 0 && xx < mStackArrayDim.at(XX))		//x direction
				{
					moveStage_({ xx,yy });
					acqStack_(iterWL);
					saveStack_();

					xx += mScanDir.at(XX);		//Increase the iterator x
				}

				//Initialize the next cycle by going back in x one step and switching the scanning direction
				xx -= mScanDir.at(XX);
				reverseStageScanDirection_(XX);

				yy += mScanDir.at(YY);	//Increase the iterator y
			}
			//Initialize the next cycle by going back in y one step and switching the scanning direction
			yy -= mScanDir.at(YY);
			reverseStageScanDirection_(YY);
		}

		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

Commandline Sequencer::getCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

void Sequencer::printSequencerParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan directions (x,y,z) = {" << mInitialScanDir.at(XX) << "," << mInitialScanDir.at(YY) << "," << mInitialScanDir.at(ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Z-stage initial position (mm) = " << mSample.mInitialZ / mm << "\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "# vibratome slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (x,y) = (" << mStackArrayDim.at(XX) << "," << mStackArrayDim.at(YY) << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";
	*fileHandle << "\n";
}

//Print the commandlist to file
void Sequencer::printToFile(const std::string fileName) const
{
	std::ofstream *fileHandle = new std::ofstream(folderPath + fileName + ".txt");

	*fileHandle << std::fixed;	//Show a fixed number of digits

	mSample.printParams(fileHandle);
	mLaserList.printParams(fileHandle);
	printSequencerParams(fileHandle);
	mStack.printParams(fileHandle);
	mVibratome.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
	//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mStackArrayDim.at(XX) * mStackArrayDim.at(YY)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}
#pragma endregion "sequencer"