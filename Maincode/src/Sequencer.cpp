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
	double scanZf_mm, scanPf_mW;

	switch (mAction)
	{
	case MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mCommand.moveStage.sliceNumber << "\t(" <<
			mCommand.moveStage.stackIJ.at(XX) << "," << mCommand.moveStage.stackIJ.at(YY) << ")\t(" <<
			std::fixed << std::setprecision(3) << mCommand.moveStage.stackCenter_mm.at(0) << "," << mCommand.moveStage.stackCenter_mm.at(1) << ")\n";
		break;
	case ACQ:
		scanZf_mm = mCommand.acqStack.scanZi_mm + mCommand.acqStack.scanDirZ * mCommand.acqStack.stackDepth_um / 1000;
		scanPf_mW = mCommand.acqStack.scanPi_mW + mCommand.acqStack.scanDirZ * mCommand.acqStack.stackPinc_mW;
		*fileHandle << actionToString_(mAction) << "\t\t\t\t\t" << mCommand.acqStack.stackNumber << "\t" << mCommand.acqStack.wavelength_nm << "\t" << mCommand.acqStack.scanDirZ << "\t" <<
			std::setprecision(3) << mCommand.acqStack.scanZi_mm << "\t" << scanZf_mm << "\t" <<
			std::setprecision(0) << mCommand.acqStack.scanPi_mW << "\t" << scanPf_mW << "\n";
		break;
	case SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case CUT:
		*fileHandle << actionToString_(mAction) << std::setprecision(3)  << "\t**************************Sample facing the vibratome at (mm) = (" <<
			mCommand.cutSlice.samplePosition_mm.at(XX) << "," << mCommand.cutSlice.samplePosition_mm.at(YY) << "," << mCommand.cutSlice.samplePosition_mm.at(ZZ) <<
			") *******************************\n";
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
		std::cout << "Vibratome slice number = " << mCommand.moveStage.sliceNumber << "\n";
		std::cout << "Stack ij = (" << mCommand.moveStage.stackIJ.at(XX) << "," << mCommand.moveStage.stackIJ.at(YY) << ")\n";
		std::cout << "Stack center (mm,mm) = (" << mCommand.moveStage.stackCenter_mm.at(XX) << "," << mCommand.moveStage.stackCenter_mm.at(YY) << ")\n" << std::endl;
		break;
	case ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength (nm) = " << mCommand.acqStack.wavelength_nm << "\n";
		std::cout << "scanDirZ = " << mCommand.acqStack.scanDirZ << "\n";
		std::cout << "scanZi/stackDepth (mm) = " << mCommand.acqStack.scanZi_mm << "/" << mCommand.acqStack.stackDepth_um / 1000 << "\n";
		std::cout << "scanPi/stackPdiff (mW) = " << mCommand.acqStack.scanPi_mW << "/" << mCommand.acqStack.stackPinc_mW << "\n" << std::endl;
		break;
	case SAV:
		std::cout << "The command is " << actionToString_(mAction) << " with no parameters" << std::endl;
		break;
	case CUT:
		std::cout << "The command is " << actionToString_(mAction) << " with no parameters" << std::endl;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequencer"
Sequencer::Sequencer(const SampleConfig sample, const std::vector<LaserConfig> laserConfig, const StackConfig stackConfig, const VibratomeConfig vibratomeConfig, const StageConfig stageConfig) :
	mSample(sample), mLaserConfig(laserConfig), mStackConfig(stackConfig), mVibratomeConfig(vibratomeConfig), mStageConfig(stageConfig)
{
	//Initialize the z-stage
	mScanZi_mm = mStageConfig.stageInitialZ_mm;

	//Initialize the height of the plane to be cut
	mPlaneToCutZ_mm = mScanZi_mm + (mVibratomeConfig.sliceThickness_um - mVibratomeConfig.sliceOffsetZ_um) / 1000;

	//Calculate the total number of stacks per vibratome slice and in the entire sample
	mStackArrayDim.at(XX) = static_cast<int>(std::ceil(1000 * mSample.length_mm.at(XX) / mStackConfig.FOV_um.at(XX)));							//Number of stacks in x
	mStackArrayDim.at(YY) = static_cast<int>(std::ceil(1000 * mSample.length_mm.at(YY) / mStackConfig.FOV_um.at(YY)));							//Number of stacks in y
	const int mNtotalStacksPerVibratomeSlice = mStackArrayDim.at(XX) * mStackArrayDim.at(YY);											//Total number of stacks in a vibratome slice
	const int mNtotalStackEntireSample = mNtotalSlices * static_cast<int>(mLaserConfig.size()) * mNtotalStacksPerVibratomeSlice;		//Total number of stacks in the entire sample

	//Pre-reserve a memory block assuming 3 actions for every stack in a vibratome slice (MOV, ACQ, and SAV); and then CUT
	mCommandList.reserve(3 * mNtotalStackEntireSample + mNtotalSlices - 1);
}

//The initial laser power of a stack-scan depends on whether the stack is imaged from the top down or from the bottom up.
//Give the laser power at the top of the stack scanPmin_mW, the power increment stackPinc_mW, and the scanning direction scanDirZ to return the initial laser power of the scan
double Sequencer::calculateStackScanInitialP_mW_(const double scanPmin_mW, const double stackPinc_mW, const int scanDirZ)
{
	if (scanDirZ > 0)	//z-stage scans upwards
		return scanPmin_mW;
	else				//z-stage scans downwards
		return scanPmin_mW + stackPinc_mW;
}

double2 Sequencer::stackIndicesToStackCenter_mm_(const int2 stackArrayIndices) const
{
	double2 stagePosition_mm;
	stagePosition_mm.at(XX) = mStackConfig.FOV_um.at(XX) / 1000 * (stackArrayIndices.at(XX) + 0.5);	// (stackIJ + 0.5) ranges from 0.5 to (mStackArrayDim - 0.5)
	stagePosition_mm.at(YY) = mStackConfig.FOV_um.at(YY) / 1000 * (stackArrayIndices.at(YY) + 0.5);	// (stackIJ + 0.5) ranges from 0.5 to (mStackArrayDim - 0.5)

	return stagePosition_mm;
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
	mScanDir = initialScanDir;
}

void Sequencer::moveStage_(const int2 stackIJ)
{
	const double2 stackCenter_mm = stackIndicesToStackCenter_mm_(stackIJ);

	Commandline commandline;
	commandline.mAction = MOV;
	commandline.mCommand.moveStage = { mSliceCounter, stackIJ, stackCenter_mm };
	mCommandList.push_back(commandline);

	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int iterWL)
{
	const double scanPi_mW = calculateStackScanInitialP_mW_(mLaserConfig.at(iterWL).scanPi_mW, mLaserConfig.at(iterWL).stackPinc_mW, mScanDir.at(ZZ));

	Commandline commandline;
	commandline.mAction = ACQ;
	commandline.mCommand.acqStack = { mStackCounter, mLaserConfig.at(iterWL).wavelength_nm, mScanDir.at(ZZ), mScanZi_mm, mStackConfig.stackDepth_um, scanPi_mW, mLaserConfig.at(iterWL).stackPinc_mW };
	mCommandList.push_back(commandline);

	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	mScanZi_mm += mScanDir.at(ZZ) * mStackConfig.stackDepth_um / 1000;		//Next initial z-scan position
	reverseStageScanDirection_(ZZ);											//Switch the scanning direction in z
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
	//Move the sample here to face the vibratome blade. Notice the additional offset in z
	const double3 samplePosition_mm = { mVibratomeConfig.samplePosition_mm.at(XX), mVibratomeConfig.samplePosition_mm.at(YY), mPlaneToCutZ_mm + mVibratomeConfig.bladeOffsetZ_um / 1000 };

	Commandline commandline;
	commandline.mAction = CUT;
	commandline.mCommand.cutSlice = { samplePosition_mm };
	mCommandList.push_back(commandline);

	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	mScanZi_mm += mStackConfig.stackDepth_um / 1000;			//Increae the height of the z-stage for the next iteration		
	mPlaneToCutZ_mm += mStackConfig.stackDepth_um / 1000;		//Increase the height of the plane to be cut for the next iteration
}

//Snake scanning
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";

	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		int xx = 0, yy = 0;				//Reset the stack indices after every cut
		resetStageScanDirections_();	//Reset the stage scan directions

		for (std::vector<int>::size_type iterWL = 0; iterWL != mLaserConfig.size(); iterWL++)
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

void Sequencer::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "# vibratome slices = " << mNtotalSlices << "\n";
	*fileHandle << "StackArray dim (x, y) = (" << mStackArrayDim.at(XX) << "," << mStackArrayDim.at(YY) << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n\n";
}

//Print the commandlist to file
void Sequencer::printToFile(const std::string fileName) const
{
	std::ofstream *fileHandle = new std::ofstream(folderPath + fileName + ".txt");

	mSample.printParams(fileHandle);
	printParams(fileHandle);
	mStackConfig.printParams(fileHandle);
	mVibratomeConfig.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 2*(3 * 3 * mStackArrayDim.at(XX) * mStackArrayDim.at(YY)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}
	fileHandle->close();
}
#pragma endregion "sequencer"