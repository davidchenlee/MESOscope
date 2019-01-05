#include "Devices.h"

#pragma region "Image"
Image::Image(FPGAns::RTcontrol &RTcontrol) :
	mRTcontrol(RTcontrol), mTiff(mRTcontrol.mWidthPerFrame_pix, mRTcontrol.mHeightPerFrame_pix, mRTcontrol.mNframes)
{
	mBufArrayA = new U32[mRTcontrol.mNpixAllFrames];
	mBufArrayB = new U32[mRTcontrol.mNpixAllFrames];
}

Image::~Image()
{
	//Stop FIFOOUTpc. Before I implemented this function, the computer crashed if the code was executed right after an exceptional termination.
	//(I think) this is because the access to FIFOOUT used to remain open and clashed with the next call
	stopFIFOOUTpc_();

	delete[] mBufArrayA;
	delete[] mBufArrayB;
	//std::cout << "Image destructor called\n";
}

//Establish a connection between FIFOOUTpc and FIFOOUTfpga
void Image::startFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

	//Flush any residual data in FIFOOUT from the previous run just in case
	FIFOOUTpcGarbageCollector_();
}

//Flush the residual data in FIFOOUTpc from the previous run, if any
void Image::FIFOOUTpcGarbageCollector_() const
{
	const U32 timeout_ms = 100;
	const int bufSize = 10000;

	U32 dummy;
	U32* garbage = new U32[bufSize];
	U32 nElemToReadA, nElemToReadB;					//Elements to read from FIFOOUTpc A and B
	int nElemTotalA = 0, nElemTotalB = 0; 			//Total number of elements read from FIFOOUTpc A and B
	while (true)
	{
		//Check if there are elements in FIFOOUTpc
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, 0, timeout_ms, &nElemToReadA));
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, 0, timeout_ms, &nElemToReadB));
		//std::cout << "FIFOOUTpc cleanup A/B: " << nElemToReadA << "/" << nElemToReadB << "\n";
		//getchar();

		if (nElemToReadA == 0 && nElemToReadB == 0)
			break;

		if (nElemToReadA > 0)
		{
			nElemToReadA = min(bufSize, nElemToReadA);	//Min between bufSize and nElemToReadA
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, nElemToReadA, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalA += nElemToReadA;
		}
		if (nElemToReadB > 0)
		{
			nElemToReadB = min(bufSize, nElemToReadB);	//Min between bufSize and nElemToReadB
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, nElemToReadB, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalB += nElemToReadB;
		}
	}
	if (nElemTotalA > 0 || nElemTotalB > 0)
		std::cout << "FIFOOUTpc garbage collector called. Number of elements cleaned up in FIFOOUTpc A/B: " << nElemTotalA << "/" << nElemTotalB << "\n";
}

//Configure FIFOOUTpc. According to NI, this step is optional
void Image::configureFIFOOUTpc_(const U32 depth) const
{
	U32 actualDepth;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << "\n";
}

//Stop the connection between FIFOOUTpc and FIFOOUTfpga
void Image::stopFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
	//std::cout << "stopFIFO called\n";
}

//Read the data in FIFOOUTpc
void Image::readFIFOOUTpc_()
{
	//TODO: save the data concurrently
	//I ran a test and found that two 32-bit FIFOOUTfpga have a larger bandwidth than a single 64 - bit FIFOOUTfpga
	//Test if the bandwidth can be increased by using 'NiFpga_AcquireFifoReadElementsU32'.Ref: http://zone.ni.com/reference/en-XX/help/372928G-01/capi/functions_fifo_read_acquire/
	//pass an array to a function: https://stackoverflow.com/questions/2838038/c-programming-malloc-inside-another-function
	//review of pointers and references in C++: https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html

	/*
	//Declare and start a stopwatch [2]
	double duration;
	auto t_start = std::chrono::high_resolution_clock::now();
	*/
	
	const int readFifoWaitingTime_ms = 5;	//Waiting time between each iteration
	const U32 timeout_ms = 100;				//FIFOOUTpc timeout
	
	//Null reading timeout
	int timeout_iter = 200;					//Timeout the whileloop if the data transfer fails
	int nullReadCounterA = 0;
	int nullReadCounterB = 0;

	//FIFOOUT
	int nElemTotalA = 0; 					//Total number of elements read from FIFOOUTpc A
	int nElemTotalB = 0; 					//Total number of elements read from FIFOOUTpc B
	
	while (nElemTotalA < mRTcontrol.mNpixAllFrames || nElemTotalB < mRTcontrol.mNpixAllFrames)
	{
		Sleep(readFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time for max transfer bandwidth

		readChunk_(nElemTotalA, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, mBufArrayA, nullReadCounterA);	//FIFOOUTpc A
		readChunk_(nElemTotalB, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, mBufArrayB, nullReadCounterB);	//FIFOOUTpc B

		if (nullReadCounterA > timeout_iter && nullReadCounterB > timeout_iter)
			throw ImageException((std::string)__FUNCTION__ + ": FIFO null-reading timeout");

		//std::cout << "FIFO A: " << nElemTotalA << "\tFIFO B: " << nElemTotalB << "\n";	//For debugging
		//std::cout << "nullReadCounter A: " << nullReadCounterA << "\tnullReadCounter: " << nullReadCounterB << "\n";	//For debugging
	}

	/*
	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << "\n";
	std::cout << "FIFOOUT bandwidth: " << 2 * 32 * mRTcontrol.mNpixAllFrames / duration / 1000 << " Mbps" << "\n"; //2 FIFOOUTs of 32 bits each
	std::cout << "Total of elements read: " << nElemTotalA << "\t" << nElemTotalB << "\n"; //Print out the total number of elements read
	*/
	

	//If all the expected data is NOT read successfully
	if (nElemTotalA <mRTcontrol.mNpixAllFrames || nElemTotalB < mRTcontrol.mNpixAllFrames)
		throw ImageException((std::string)__FUNCTION__ + ": Received less FIFO elements than expected");
}

//Read a chunk of data in the FIFOpc
void Image::readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, U32* buffer, int &nullReadCounter)
{
	U32 dummy;
	U32 nElemToRead = 0;				//Elements remaining in FIFOOUTpc
	const U32 timeout_ms = 100;			//FIFOOUTpc timeout

	if (nElemRead < mRTcontrol.mNpixAllFrames)		//Skip if all the data have already been transferred
	{
		//By requesting 0 elements from FIFOOUTpc, the function returns the number of elements available. If no data is available, nElemToRead = 0 is returned
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), FIFOOUTpc, buffer, 0, timeout_ms, &nElemToRead));
		//std::cout << "Number of elements remaining in FIFOOUT: " << nElemToRead << "\n";	//For debugging

		//If data available in FIFOOUTpc, retrieve it
		if (nElemToRead > 0)
		{
			//If more data than expected
			if (static_cast<int>(nElemRead + nElemToRead) > mRTcontrol.mNpixAllFrames)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Received more FIFO elements than expected");

			//Retrieve the elements in FIFOOUTpc
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getFpgaHandle(), FIFOOUTpc, buffer + nElemRead, nElemToRead, timeout_ms, &dummy));

			//Keep track of the total number of elements read
			nElemRead += nElemToRead;

			nullReadCounter = 0;	//Reset the iteration counter
		}
		else
			nullReadCounter++;		//keep track of the null reads
	}
}


//The RS scans bidirectionally. The pixel order has to be reversed for every odd line.
void Image::correctInterleaved_()
{
	//Within an odd line, the pixels go from lineIndex*widthPerFrame_pix to lineIndex*widthPerFrame_pix + widthPerFrame_pix - 1
	for (int lineIndex = 1; lineIndex < mRTcontrol.mHeightAllFrames_pix; lineIndex += 2)
		std::reverse(mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix, mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix + mRTcontrol.mWidthPerFrame_pix);
}

//Once multiplexing is implemented, each U32 element in bufArray_B must be deMux in 8 segments of 4-bits each
void Image::demultiplex_()
{
	for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixAllFrames; pixIndex++)
	{
		unsigned char upscaled = mRTcontrol.mUpscaleU8 * mBufArrayB[pixIndex]; //Upscale the buffer to go from 4-bit to 8-bit

		//If upscaled overflows
		if (upscaled > _UI8_MAX)
			upscaled = _UI8_MAX;

		//Transfer the result to Tiff
		(mTiff.pointerToTiff())[pixIndex] = upscaled;
	}
}

void Image::acquire()
{
	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to the FPGA
	startFIFOOUTpc_();				//Establish the connection between FIFOOUTfpga and FIFOOUTpc and cleans up any residual data from the previous run
	mRTcontrol.triggerRT();			//Trigger the RT control. If triggered too early, FIFOOUTfpga will probably overflow

	if (FIFOOUTfpga)
	{
		try
		{
			readFIFOOUTpc_();			//Read the data received in FIFOOUTpc
			correctInterleaved_();
			demultiplex_();				//Move the chuncks of data to the buffer array
		}
		catch (const ImageException &e) //Notify the exception and continue with the next iteration
		{
			std::cerr << "An ImageException has occurred in: " << e.what() << "\n";
		}
	}
}

void Image::initialize()
{
	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to the FPGA
}

void Image::startFIFOOUTpc()
{
	startFIFOOUTpc_();				//Establish the connection between FIFOOUTfpga and FIFOOUTpc and cleans up any residual data from the previous run
}

void Image::download()
{
	if (FIFOOUTfpga)
	{
		try
		{
			readFIFOOUTpc_();			//Read the data received in FIFOOUTpc
			correctInterleaved_();
			demultiplex_();				//Move the chuncks of data to the buffer array
		}
		catch (const ImageException &e) //Notify the exception and continue with the next iteration
		{
			std::cerr << "An ImageException has occurred in: " << e.what() << "\n";
		}
	}
}



//The galvo (vectical axis of the image) performs bi-directional scanning
//Divide the long vertical image in nFrames and vertically mirror the odd frames
void Image::mirrorOddFrames()
{
	mTiff.mirrorOddFrames();
}

//Split the long vertical image into nFrames and calculate the average
void Image::average()
{
	mTiff.average();
}

//Save each frame in mTiff in a single Tiff page
void Image::saveTiffSinglePage(std::string filename, const OverrideFileSelector overrideFlag, const StackScanDir stackScanDir) const
{
	mTiff.saveToFile(filename, SINGLEPAGE, overrideFlag, stackScanDir);
}

//Save each frame in mTiff in a different Tiff page
void Image::saveTiffMultiPage(std::string filename, const OverrideFileSelector overrideFlag, const StackScanDir stackScanDir) const
{
	mTiff.saveToFile(filename, MULTIPAGE, overrideFlag, stackScanDir);
}

//Access the Tiff data in the Image object
unsigned char* const Image::pointerToTiff() const
{
	return mTiff.pointerToTiff();
}
#pragma endregion "Image"


#pragma region "Vibratome"
Vibratome::Vibratome(const FPGAns::FPGA &fpga): mFpga(fpga){}

//Start or stop running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::pushStartStopButton() const
{
	const double SleepTime = 20 * ms; //in ms. It has to be ~ 12 ms or longer otherwise the vibratome control pad does not read the signal
	
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTstart, true));

	Sleep(static_cast<DWORD>(SleepTime/ms));

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
}

//Move the head of the vibratome forward or backward for 'duration'. The timing varies in approx 1 ms
void Vibratome::moveHead_(const double duration, const MotionDir motionDir) const
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const double minDuration = 10 * ms;
	const double delay = 1 * ms;				//Used to roughly calibrate the pulse length

	switch (motionDir)
	{
	case BACKWARD:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VTback;
		break;
	case FORWARD:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VTforward;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected vibratome channel unavailable");
	}

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), selectedChannel, true));

	if (duration >= minDuration)
		Sleep(static_cast<DWORD>((duration - delay)/ms));
	else
	{
		Sleep(static_cast<DWORD>((minDuration - delay)/ms));
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << 1. * minDuration / ms << "ms" << "\n";
	}
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), selectedChannel, false));
}

void Vibratome::cutAndRetractDistance(const double distance) const
{
	const double cuttingTime = distance / mCuttingSpeed;
	const double retractingTime = distance / mMovingSpeed;

	pushStartStopButton();
	std::cout << "The vibratome is cutting for " << cuttingTime / sec << " seconds" << "\n";
	Sleep(static_cast<DWORD>(cuttingTime / ms));
	Sleep(2000);
	std::cout << "The vibratome is retracting for " << retractingTime / sec << " seconds" << "\n";
	moveHead_(retractingTime, BACKWARD);
}


void Vibratome::retractDistance(const double distance) const
{
	const double retractingTime = static_cast<int>(distance / mMovingSpeed);
	std::cout << "The vibratome is retracting for " << retractingTime / sec << " seconds" << "\n";
	moveHead_(retractingTime, BACKWARD);
}


#pragma endregion "Vibratome"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const FPGAns::RTcontrol &RTcontrol): mRTcontrol(RTcontrol)
{	
	//Calculate the spatial fill factor
	const double temporalFillFactor = mRTcontrol.mWidthPerFrame_pix * mRTcontrol.mDwell / halfPeriodLineclock;
	if (temporalFillFactor > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");
	else
		mFillFactor = sin(PI / 2 * temporalFillFactor);						//Note that the fill factor doesn't depend on the RS amplitude because the RS period is fixed

	//std::cout << "Fill factor = " << mFillFactor << "\n";					//For debugging

	//Download the current control voltage from the FPGA and update the scan parameters
	mControlVoltage = downloadControlVoltage();								//Control voltage
	mFullScan = mControlVoltage / mVoltagePerDistance;						//Full scan FOV = distance between the turning points
	mFFOV = mFullScan * mFillFactor;										//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;						//Spatial sampling resolution (length/pixel)
}

//Set the control voltage that determines the scanning amplitude
void ResonantScanner::setVoltage_(const double controlVoltage)
{
	if (controlVoltage < 0 || controlVoltage > mVMAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage must be in the range 0-" + std::to_string(mVMAX) + " V" );

	//Update the scan parameters
	mControlVoltage = controlVoltage;									//Control voltage
	mFullScan = controlVoltage / mVoltagePerDistance;					//Full scan FOV
	mFFOV = mFullScan * mFillFactor;									//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;					//Spatial sampling resolution (length/pixel)

	//Upload the control voltage
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::convertVoltageToI16(mControlVoltage)));
}

//Set the full FOV of the microscope. FFOV does not include the cropped out areas at the turning points
void ResonantScanner::setFFOV(const double FFOV)
{
	//Update the scan parameters
	mFullScan = FFOV / mFillFactor;										//Full scan FOV
	mControlVoltage = mFullScan * mVoltagePerDistance;					//Control voltage
	mFFOV = FFOV;														//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;					//Spatial sampling resolution (length/pixel)
	//std::cout << "mControlVoltage = " << mControlVoltage << "\n";		//For debugging

	if (mControlVoltage < 0 || mControlVoltage > mVMAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested FFOV must be in the range 0-" + std::to_string(mVMAX/mVoltagePerDistance /um) + " um");

	//Upload the control voltage
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::convertVoltageToI16(mControlVoltage)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn(const double FFOV)
{
	setFFOV(FFOV);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV / um << " um\n";
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOnUsingVoltage(const double controlVoltage)
{
	setVoltage_(controlVoltage);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << controlVoltage / V << " V\n";
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(static_cast<DWORD>(mDelay/ms));
	setVoltage_(0);
	std::cout << "RS successfully turned off" << "\n";
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControlVoltage() const
{
	I16 control_I16;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadI16((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

	return FPGAns::convertI16toVoltage(control_I16);
}

//Check if the RS is set to run. It does not actually check if the RS is running, for example, by looking at the RSsync signal
void ResonantScanner::isRunning() const
{
	//Retrieve the state of the RS from the FPGA (see the LabView implementation)
	NiFpga_Bool isRunning;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadBool((mRTcontrol.mFpga).getFpgaHandle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));

	char input_char;
	while (!isRunning)
	{
		std::cout << "RS seems OFF. Input 0 to exit or any other key to try again ";
		std::cin >> input_char;

		if (input_char == '0')
			throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
	}
}

#pragma endregion "Resonant scanner"


#pragma region "Galvo"

Galvo::Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel):
	mRTcontrol(RTcontrol), mGalvoRTchannel(galvoChannel)
{
	if ( mGalvoRTchannel != RTGALVO1 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
}

void Galvo::pushVoltageSinglet(const double timeStep, const double AO) const
{
	mRTcontrol.pushAnalogSinglet(mGalvoRTchannel, timeStep, AO);
}

void Galvo::voltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf) const
{
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, Vi, Vf);
}

void Galvo::positionLinearRamp(const double timeStep, const double rampLength, const double xi, const double xf) const
{
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, mVoltagePerDistance * xi, mVoltagePerDistance * xf);
}

void Galvo::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mGalvoRTchannel, AO_tMIN, 0 * V);
}
#pragma endregion "Galvo"


#pragma region "PMT16X"
PMT16X::PMT16X()
{
	mSerial = new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout/ms));
}

PMT16X::~PMT16X()
{
	mSerial->close();
}

std::vector<uint8_t> PMT16X::sendCommand_(std::vector<uint8_t> command_array) const
{
	command_array.push_back(sumCheck_(command_array, command_array.size()));	//Append the sumcheck

	std::string TxBuffer(command_array.begin(), command_array.end()); //Convert the vector<char> to string
	TxBuffer += "\r";	//End the command line with CR
	//printHex(TxBuffer); //For debugging

	std::vector<uint8_t> RxBuffer;
	mSerial->write("\r");						//Wake up the PMT16X
	mSerial->read(RxBuffer, mRxBufferSize);		//Read the state: 0x0D(0d13) for ready, or 0x45(0d69) for error

	//Throw an error if RxBuffer is empty or CR is NOT returned
	if ( RxBuffer.empty() || RxBuffer.at(0) != 0x0D )
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure waking up the PMT16X microcontroller");
	
	//printHex(RxBuffer); //For debugging

	RxBuffer.clear(); //Flush the buffer
	mSerial->write(TxBuffer);
	mSerial->read(RxBuffer, mRxBufferSize);
	
	//Throw an error if RxBuffer is empty
	if (RxBuffer.empty())
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure reading the PMT16X microcontroller");

	//printHex(RxBuffer); //For debugging

	return RxBuffer;
}

//Return the sumcheck of all the elements in the array
uint8_t PMT16X::sumCheck_(const std::vector<uint8_t> charArray, const int nElements) const
{
	uint8_t sum = 0;
	for (int ii = 0; ii < nElements; ii++)
		sum += charArray.at(ii);

	return sum;
}

void PMT16X::readAllGain() const
{
	std::vector<uint8_t> parameters = sendCommand_({'I'});

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'I' || parameters.at(17) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__  + ": CheckSum mismatch\n";
	
	//Print out the gains
	std::cout << "PMT16X gains:\n";
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << static_cast<int>(parameters.at(ii)) << "\n";		
}

void PMT16X::setSingleGain(const int channel, const int gain) const
{
	//Check that the inputVector parameters are within range
	if (channel < 1 || channel > 16)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X channel number out of range (1-16)");

	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain out of range (0-255)");


	std::vector<uint8_t> parameters = sendCommand_({'g', (uint8_t)channel, (uint8_t)gain});
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'g' && parameters.at(1) == (uint8_t)channel && parameters.at(2) == (uint8_t)gain && parameters.at(3) == sumCheck_(parameters, parameters.size()-2))
		std::cout << "PMT16X channel " << channel << " successfully set to " << gain << "\n";
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";
}

void PMT16X::setAllGainToZero() const
{
	std::vector<uint8_t> parameters = sendCommand_({ 'R' }); //The manual says that this sets all the gains to 255, but it really does it to 0
															//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. The second char returned is the sumcheck
	if (parameters.at(0) == 'R' && parameters.at(1) == 'R')
		std::cout << "All PMT16X gains successfully set to 0\n";
}

void PMT16X::setAllGain(const int gain) const
{
	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain must be in the range 0-255");

	std::vector<uint8_t> parameters = sendCommand_({ 'S', (uint8_t)gain });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'S' && parameters.at(1) == (uint8_t)gain && parameters.at(2) == sumCheck_(parameters, parameters.size() - 2))
		std::cout << "All PMT16X gains successfully set to " << gain << "\n";
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";
}

void PMT16X::setAllGain(std::vector<uint8_t> gains) const
{
	//Check that the inputVector parameters are within range
	if (gains.size() != 16)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Gain array must have 16 elements");

	for (int ii = 0; ii < 16; ii++)
		if (gains.at(ii) < 0 || gains.at(ii) > 255)
			throw std::invalid_argument((std::string)__FUNCTION__ + ":  PMT16X gain #" + std::to_string(ii) + " out of range (0-255)");

	gains.insert(gains.begin(), 'G');	//Prepend the command
	std::vector<uint8_t> parameters = sendCommand_({ gains });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'G' || parameters.at(17) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	//Print out the gains
	std::cout << "PMT16X gains successfully set to:\n";
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << static_cast<int>(parameters.at(ii)) << "\n";

}

void PMT16X::readTemp() const
{
	std::vector<uint8_t> parameters = sendCommand_({ 'T' });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'T' || parameters.at(4) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	const int TEMPH = static_cast<int>(parameters.at(1));
	const int TEMPL = static_cast<int>(parameters.at(2));
	const double temp_C = TEMPH + 0.01 * TEMPL; //According to the manual

	const int alertTemp_C = static_cast<int>(parameters.at(3));

	std::cout << "PMT16X temperature = " << temp_C << " \370C\n";
	std::cout << "PMT16X alert temperature = " << alertTemp_C <<  " \370C\n";
}
#pragma endregion "PMT16X"


#pragma region "Filterwheel"
Filterwheel::Filterwheel(const FilterwheelSelector whichFilterwheel): mWhichFilterwheel(whichFilterwheel)
{
	switch (whichFilterwheel)
	{
	case FWEXC:
		mPort = COMFWEXC;
		mFilterwheelName = "Excitation filterwheel";
		mFWconfig = mExcConfig;								//Assign the filter positions
		break;
	case FWDET:
		mPort = COMFWDET;
		mFilterwheelName = "Detection filterwheel";
		mFWconfig = mDetConfig;								//Assign the filter positions
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected filterwheel unavailable");
	}

	try
	{
		mSerial = new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms));
		downloadColor_();	//Download the current filter position
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mFilterwheelName);
	}
}

Filterwheel::~Filterwheel()
{
	mSerial->close();
}

//Download the current filter position
void Filterwheel::downloadColor_()
{
	const std::string TxBuffer("pos?\r");	//Command to the filterwheel
	std::string RxBuffer;					//Reply from the filterwheel

	try
	{
		mSerial->write(TxBuffer);
		mSerial->read(RxBuffer, mRxBufSize);
		//std::cout << "Full RxBuffer: " << RxBuffer << "\n"; //For debugging

		//Delete echoed command
		std::string::size_type ii = RxBuffer.find(TxBuffer);
		if (ii != std::string::npos)
			RxBuffer.erase(ii, TxBuffer.length());

		//Delete CR and >
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
		//RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

		//std::cout << "RxBuffer: " << RxBuffer << "\n"; //For debugging
		mPosition = std::stoi(RxBuffer);									//convert string to int
		mColor = convertPositionToColor_(mPosition);
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mFilterwheelName);
	}
}

void Filterwheel::setPosition_(const int position)
{
	if (position != mPosition)
	{
		std::string TxBuffer("pos=" + std::to_string(position) + "\r");
		std::string RxBuffer;

		try
		{
			mSerial->write(TxBuffer);

			//Find the shortest way to reach the targeted position
			const int minPos = (std::min)(position, mPosition);
			const int maxPos = (std::max)(position, mPosition);
			const int diffPos = maxPos - minPos;
			const int minSteps = (std::min)(diffPos, mNpos - diffPos);

			//std::cout << "Tuning the " << mDeviceName << " to " + convertColorToString_(color) << "...\n";
			Sleep(static_cast<DWORD>(1. * minSteps / mTuningSpeed / ms));	//Wait until the filterwheel stops turning the turret

			mSerial->read(RxBuffer, mRxBufSize);		//Read RxBuffer to flush it. Serial::flush() doesn't work
														//std::cout << "setColor full RxBuffer: " << RxBuffer << "\n"; //For debugging

			downloadColor_();	//Check if the filterwheel was set successfully 

			if (position == mPosition)
				std::cout << mFilterwheelName << " successfully set to " + convertColorToString_(mColor) << " (position = " << mPosition << ")\n";
			else
				std::cout << "WARNING: " << mFilterwheelName << " might not be in the correct position " << position << "\n";
		}
		catch (const serial::IOException)
		{
			throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mFilterwheelName);
		}
	}
}

int Filterwheel::convertColorToPosition_(const Filtercolor color) const
{
	for (std::vector<int>::size_type iter = 0; iter != mFWconfig.size(); iter++)
	{
		if (color == mFWconfig.at(iter))
			return iter + 1;			//The index for mFWconfig starts from 0. The index for the filterwheel position start from 1
	}
	
	throw std::runtime_error((std::string)__FUNCTION__ + ": Failure converting color to position");
}

Filtercolor Filterwheel::convertPositionToColor_(const int position) const
{
	if (position < 1 || position > mNpos)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": the filterwheel position must be between 1 and " + std::to_string(mNpos));

	return mFWconfig.at(position - 1);
}

//Convert from enum Filtercolor to string
std::string Filterwheel::convertColorToString_(const Filtercolor color) const
{
	std::string colorStr;
	switch (color)
	{
	case BLUE:
		colorStr = "BLUE";
		break;
	case GREEN:
		colorStr = "GREEN";
		break;
	case RED:
		colorStr = "RED";
		break;
	case NONE:
		colorStr = "NONE";
		break;
	default:
		colorStr = "UNKNOWN";
	}
	return colorStr;
}

void Filterwheel::setColor(const Filtercolor color)
{
	setPosition_(convertColorToPosition_(color));
}

//Set the filter color using the laser wavelength
void Filterwheel::setColor(const int wavelength_nm)
{
	Filtercolor color;
	//Wavelength intervals chosen based on the 2p-excitation spectrum of the fluorescent labels (DAPI, GFP, and tdTomato)
	if (wavelength_nm > 940)
		color = RED;
	else if (wavelength_nm > 790)
		color = GREEN;
	else
		color = BLUE;

	setPosition_(convertColorToPosition_(color));
}
#pragma endregion "Filterwheel"

#pragma region "Laser"
Laser::Laser(const LaserSelector laserID): mWhichLaser(laserID)
{
	switch (mWhichLaser)
	{
	case VISION:
		laserName = "VISION";
		mPort = COMVISION;
		mBaud = 19200;
		break;
	case FIDELITY:
		laserName = "FIDELITY";
		mPort = COMFIDELITY;
		mBaud = 115200;
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial = new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms));
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure establishing serial communication with " + laserName);
	}

	mWavelength_nm = downloadWavelength_nm_();
}

Laser::~Laser()
{
	mSerial->close();
}

int Laser::downloadWavelength_nm_()
{
	switch (mWhichLaser)
	{
	case VISION:
		try
		{
			const std::string TxBuffer("?VW");		//Command to the laser
			std::string RxBuffer;					//Reply from the laser
			mSerial->write(TxBuffer + "\r");
			mSerial->read(RxBuffer, mRxBufSize);

			//Delete echoed command. Echoing could be disabled on the laser side but deleting it is safer and more general
			std::string keyword("?VW ");
			std::string::size_type i = RxBuffer.find(keyword);
			if (i != std::string::npos)
				RxBuffer.erase(i, keyword.length());

			//Delete "CHAMELEON>". This frase could be disabled on the laser side, but deleting it is safer and more general
			keyword = "CHAMELEON>";
			i = RxBuffer.find(keyword);
			if (i != std::string::npos)
				RxBuffer.erase(i, keyword.length());

			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());
			//std::cout << RxBuffer << "\n";	//For debugging

			return std::stoi(RxBuffer);	//Convert string to int

		}
		catch (const serial::IOException)
		{
			throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
		}
	case FIDELITY:
		return 1040;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}	
}

void Laser::printWavelength_nm() const
{
	std::cout << laserName +  " wavelength is " << mWavelength_nm << " nm\n";
}

void Laser::setWavelength(const int wavelength_nm)
{
	switch (mWhichLaser)
	{
	case VISION:
		if (wavelength_nm < 680 || wavelength_nm > 1080)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": VISION wavelength must be in the range 680 - 1080 nm");

		if (wavelength_nm != mWavelength_nm)
		{
			const std::string TxBuffer("VW=" + std::to_string(wavelength_nm));		//Command to the laser
			std::string RxBuffer;													//Reply from the laser

			try
			{
				mSerial->write(TxBuffer + "\r");

				//std::cout << "Sleep time in ms: " << static_cast<int>( std::abs( 1.*(mWavelength_nm - wavelength_nm) / mTuningSpeed / ms ) ) << "\n";	//For debugging
				std::cout << "Tuning VISION to " << wavelength_nm << " nm...\n";
				Sleep(static_cast<DWORD>( std::abs( 1.*(mWavelength_nm - wavelength_nm) / mTuningSpeed / ms )) );	//Wait till the laser stops tuning

				mSerial->read(RxBuffer, mRxBufSize);	//Read RxBuffer to flush it. Serial::flush() doesn't work. The message reads "CHAMELEON>"

				downloadWavelength_nm_();				//Check if the laser was set successfully 

				if (mWavelength_nm = wavelength_nm)
					std::cout << "VISION wavelength successfully set to " << wavelength_nm << " nm\n";
				else
					std::cout << "WARNING: VISION might not be in the correct wavelength " << wavelength_nm << " nm\n";
			}
			catch (const serial::IOException)
			{
				throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
			}
		}
		break;
	case FIDELITY:
		std::cout << "WARNING: FIDELITY wavelength only supports 1040 nm\n";
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

//Open or close the internal shutter of the laser
void Laser::setShutter(const bool state) const
{
	std::string TxBuffer;		//Command to the laser
	std::string RxBuffer;		//Reply from the laser

	switch (mWhichLaser)
	{
	case VISION:
		TxBuffer = "S=" + std::to_string(state);
		break;
	case FIDELITY:
		TxBuffer = "SHUTTER=" + std::to_string(state);
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial->write(TxBuffer + "\r");
		mSerial->read(RxBuffer, mRxBufSize);	//Read RxBuffer to flush it. Serial::flush() doesn't work.

		if (state)
			std::cout << laserName + " shutter successfully opened\n";
		else
			std::cout << laserName + " shutter successfully closed\n";
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with " + laserName);
	}
}

bool Laser::isShutterOpen() const
{
	std::string TxBuffer;		//Command to the laser
	std::string RxBuffer;		//Reply from the laser
	std::string keyword;		//To delete the echo from the laser. Echoing could be disabled on the laser side but deleting it is safer and more general

	switch (mWhichLaser)
	{
	case VISION:
		TxBuffer = "?S";
		keyword = "?S ";
		break;
	case FIDELITY:
		TxBuffer = "?SHUTTER";
		keyword = "?SHUTTER\t";
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial->write(TxBuffer + "\r");
		mSerial->read(RxBuffer, mRxBufSize);	//Read RxBuffer to flush it. Serial::flush() doesn't work

		//Remove echoed command in the returned message
		std::string::size_type i = RxBuffer.find(keyword);
		if (i != std::string::npos)
			RxBuffer.erase(i, keyword.length());

		//Return a boolean
		if (RxBuffer.front() == '0')
			return false;
		else if (RxBuffer.front() == '1')
			return true;
		else
			throw std::runtime_error((std::string)__FUNCTION__ + ": Laser returned invalid shutter state");
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with " + laserName);
	}
}
#pragma endregion "Laser"


#pragma region "Shutters"
Shutter::Shutter(const FPGAns::FPGA &fpga, const LaserSelector whichLaser) : mFpga(fpga)
{
	switch (whichLaser)
	{
	case VISION:
		mWhichShutter = NiFpga_FPGAvi_ControlBool_ShutterVision;
		break;
	case FIDELITY:
		mWhichShutter = NiFpga_FPGAvi_ControlBool_ShutterFidelity;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected shutter unavailable");
	}
}

Shutter::~Shutter()
{
	//This is to prevent keeping the shutter open in case of an exception
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mWhichShutter, false));
}

void Shutter::setShutter(const bool state) const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mWhichShutter, state));
}


void Shutter::pulse(const double pulsewidth) const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mWhichShutter, true));

	Sleep(static_cast<DWORD>(pulsewidth/ms));

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mWhichShutter, false));
}
#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Curently, output of the pockels cell is hardcoded on the FPGA side.  The pockels' output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(FPGAns::RTcontrol &RTcontrol, const LaserSelector laserSelector, const int wavelength_nm) :
	mRTcontrol(RTcontrol), mWavelength_nm(wavelength_nm), mShutter(mRTcontrol.mFpga, laserSelector)
{
	if (laserSelector != VISION && laserSelector != FIDELITY)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels channel unavailable");

	switch (laserSelector)
	{
	case VISION:
		mPockelsRTchannel = RTVISION;
		mScalingRTchannel = RTSCALINGVISION;
		break;
	case FIDELITY:
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength of FIDELITY can not be different from 1040 nm");
		mPockelsRTchannel = RTFIDELITY;
		mScalingRTchannel = RTSCALINGFIDELITY;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	//Initialize all the scaling factors to 1.0. In LV, I could not sucessfully default the LUT to 0d16384 = 0b0100000000000000 = 1 for a fixed point Fx2.14
	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchannel, 1.0);
}

double PockelsCell::convertLaserpowerToVolt_(const double power) const
{
	double a, b, c;		//Calibration parameters

	//VISION
	switch (mPockelsRTchannel)
	{
	case RTVISION:
		if (mWavelength_nm == 750) {
			a = 788 * mW;
			b = 0.6152 / V;
			c = -0.027 * V;
		}
		else if (mWavelength_nm == 940) {
			a = 464 * mW;
			b = 0.488 / V;
			c = -0.087 * V;
		}
		else if (mWavelength_nm == 1040) {
			a = 200 * mW;
			b = 0.441 / V;
			c = 0.037 * V;
		}
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		break;

		//FIDELITY
	case RTFIDELITY:
		a = 101.20 * mW;
		b = 0.276 / V;
		c = -0.049 * V;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	double arg = sqrt(power / a);
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Arg of asin is greater than 1");

	return asin(arg) / b + c;
}


void PockelsCell::pushVoltageSinglet(const double timeStep, const double AO) const
{
	if (AO < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, AO);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P, const OverrideFileSelector overrideFlag) const
{
	if (P < 0 || P > maxPower)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's laser power must be in the range 0-" + std::to_string(P/mW));

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, convertLaserpowerToVolt_(P), overrideFlag);
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void PockelsCell::voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi, const double Vf) const
{
	if (Vi < 0 || Vf < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, Vi, Vf);
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void  PockelsCell::powerLinearRamp(const double timeStep, const double rampDuration, const double Pi, const double Pf) const
{
	if (Pi < 0 || Pf < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, convertLaserpowerToVolt_(Pi), convertLaserpowerToVolt_(Pf));
}

void PockelsCell::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, AO_tMIN, 0 * V);
}

//Linearly scale the pockels output across all the frames
void PockelsCell::scalingLinearRamp(const double Si, const double Sf) const
{
	if (Si < 0 || Sf < 0 || Si > 4 || Sf > 4)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested scaling factor must be in the range 0-4");

	if (mRTcontrol.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	mRTcontrol.clearQueue(mScalingRTchannel);	//Delete the default scaling factors of 1.0 created in the PockelsCell constructor

	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchannel, Si + (Sf - Si) / (mRTcontrol.mNframes - 1) * ii);
}

void PockelsCell::setShutter(const bool state) const
{
	mShutter.setShutter(state);
}

#pragma endregion "Pockels cells"


//Integrate the lasers, pockels cells, and filterwheels in a single class
#pragma region "VirtualLaser"
VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LaserSelector laserSelector) :
	mWavelength_nm(wavelength_nm), mVision(VISION), mFidelity(FIDELITY), mPockelsVision(RTcontrol, VISION, wavelength_nm), mPockelsFidelity(RTcontrol, FIDELITY, 1040), mFWexcitation(FWEXC), mFWdetection(FWDET)
{

	switch (laserSelector)
	{
	case VISION:
		mWhichLaser = VISION;
		mVision.setWavelength(wavelength_nm);
		checkShutterIsOpen_(mVision);
		break;
	case FIDELITY:
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength of FIDELITY can not be different from 1040 nm");

		mWhichLaser = FIDELITY;
		checkShutterIsOpen_(mFidelity);
		break;
	case AUTO:	//Use VISION for everything below 1040 nm. Use FIDELITY for 1040 nm	
		if (wavelength_nm < 1040)
		{
			mWhichLaser = VISION;
			mVision.setWavelength(wavelength_nm);
			checkShutterIsOpen_(mVision);
		}
		
		else if (wavelength_nm == 1040)
		{
			mWhichLaser = FIDELITY;
			checkShutterIsOpen_(mFidelity);
		}
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": wavelength > 1040 nm is not implemented in the VirtualLaser class");
		break;
	}

	std::cout << "Using " << laserSelectorToString_(mWhichLaser) << " at " << mWavelength_nm << " nm\n";

	//Set the filterwheels
	if (0) //Multiplexing
	{
		mFWexcitation.setColor(wavelength_nm);
	}
	else   //Single beam
		mFWexcitation.setColor(NONE);

	mFWdetection.setColor(wavelength_nm);
}

void VirtualLaser::setWavelength_(const int wavelength_nm)
{
	//Use VISION for everything below 1040 nm
	if (wavelength_nm < 1040)
	{
		mWhichLaser = VISION;
		mVision.setWavelength(wavelength_nm);
	}
	//Use FIDELITY for 1040 nm
	else if (wavelength_nm == 1040)
		mWhichLaser = FIDELITY;
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": wavelength > 1040 nm is not implemented in the VirtualLaser class");

	mFWexcitation.setColor(wavelength_nm);
	mFWdetection.setColor(wavelength_nm);
	mWavelength_nm = wavelength_nm;
}

std::string VirtualLaser::laserSelectorToString_(const LaserSelector whichLaser) const
{
	switch (whichLaser)
	{
	case VISION:
		return "VISION";
	case FIDELITY:
		return "FIDELITY";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

void VirtualLaser::checkShutterIsOpen_(const Laser &laser) const
{
	if (!laser.isShutterOpen())
		throw std::runtime_error((std::string)__FUNCTION__ + ": The shutter of " + laser.laserName + " seems to be closed");
}

void VirtualLaser::pushPowerSinglet(const double timeStep, const double power, const OverrideFileSelector overrideFlag) const
{
	switch (mWhichLaser)
	{
	case VISION:
		mPockelsVision.pushPowerSinglet(timeStep, power, overrideFlag);
		break;
	case FIDELITY:
		mPockelsFidelity.pushPowerSinglet(timeStep, power, overrideFlag);
		break;
	}
}

void VirtualLaser::setShutter(const bool state) const
{
	switch (mWhichLaser)
	{
	case VISION:
		mPockelsVision.setShutter(state);
		break;
	case FIDELITY:
		mPockelsFidelity.setShutter(state);
		break;
	}
}

#pragma endregion "VirtualLaser"

#pragma region "Stages"
Stage::Stage(const double velX, const double velY, const double velZ)
{
	const std::string stageIDx = "116049107";	//X-stage (V-551.4B)
	const std::string stageIDy = "116049105";	//Y-stage (V-551.2B)
	const std::string stageIDz = "0165500631";	//Z-stage (ES-100)

	//Open the connections to the stage controllers and assign the IDs
	std::cout << "Establishing connection with the stages\n";
	mID.at(XX) = PI_ConnectUSB(stageIDx.c_str());
	mID.at(YY) = PI_ConnectUSB(stageIDy.c_str());
	mID.at(ZZ) = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID.at(ZZ) = PI_ConnectUSB(stageIDz.c_str());

	if (mID.at(XX) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");

	if (mID.at(YY) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");

	if (mID.at(ZZ) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	std::cout << "Connection with the stages successfully established\n";

	//Record the current position
	mPositionXYZ.at(XX) = downloadPosition(XX);
	mPositionXYZ.at(YY) = downloadPosition(YY);
	mPositionXYZ.at(ZZ) = downloadPosition(ZZ);

	//Configure the stage velocities and DO triggers
	configVelAndDOtriggers_({ velX, velY, velZ });
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mID.at(XX));
	PI_CloseConnection(mID.at(YY));
	PI_CloseConnection(mID.at(ZZ));
	std::cout << "Connection to the stages successfully closed\n";
}


//DO1 and DO2 are used to trigger the stack acquisition. Currently DO2 is used as the only trigger. See the implementation on LV
void Stage::configVelAndDOtriggers_(const double3 velXYZ) const
{
	//DO1 TRIGGER: DO1 is set to output a pulse (fixed width = 50 us) whenever the stage covers a certain distance (e.g. 0.3 um)
	const int DO1 = 1;
	setDOtriggerEnabled(ZZ, DO1, true);	//Enable DO1 trigger
	const double triggerStep = 0.3 * um;
	const StageDOtriggerMode triggerMode = PositionDist;
	const double startThreshold = 0 * mm;
	const double stopThreshold = 0 * mm;
	setDOtriggerAllParams(ZZ, DO1, triggerStep, triggerMode, startThreshold, stopThreshold);

	//DO2 TRIGGER: DO2 is set to output HIGH when the stage z is in motion
	const int DO2 = 2;
	setDOtriggerEnabled(ZZ, DO2, true);	//Enable DO2 trigger
	setDOtriggerSingleParam(ZZ, DO2, TriggerMode, InMotion);

	//Set the stage velocities
	setAllVelocities({ velXYZ.at(XX), velXYZ.at(YY), velXYZ.at(ZZ) });
}

//Recall the current position for the 3 stages
double3 Stage::readPositionXYZ() const
{
	return mPositionXYZ;
}

void Stage::printPositionXYZ() const
{
	std::cout << "Stage X position = " << mPositionXYZ.at(XX) / mm << " mm\n";
	std::cout << "Stage Y position = " << mPositionXYZ.at(YY) / mm << " mm\n";
	std::cout << "Stage Z position = " << mPositionXYZ.at(ZZ) / mm << " mm\n";
}

//Retrieve the stage position from the controller
double Stage::downloadPosition(const Axis axis)
{
	double position_mm;	//Position in mm
	if (!PI_qPOS(mID.at(axis), mNstagesPerController, &position_mm))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query position for the stage " + axisToString(axis));

	return position_mm * mm;	//Multiply by mm to convert from explicit to implicit units
}

//Move the stage to the requested position
void Stage::moveSingleStage(const Axis axis, const double position)
{
	//Check if the requested position is within range
	if (position < mSoftPosMinXYZ.at(axis) || position > mSoftPosMaxXYZ.at(axis))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested position out of bounds for stage " + axisToString(axis));

	//Move the stage
	if (mPositionXYZ.at(axis) != position ) //Move only if the requested position is different from the current position
	{
		const double position_mm = position / mm;							//Divide by mm to convert from implicit to explicit units
		if (!PI_MOV(mID.at(axis), mNstagesPerController, &position_mm) )	//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage " + axisToString(axis) + " to the target position");

		mPositionXYZ.at(axis) = position;
	}
}

//Move the 3 stages to the requested position
void Stage::moveAllStages(const double3 positionXYZ)
{
	moveSingleStage(XX, positionXYZ.at(XX));
	moveSingleStage(YY, positionXYZ.at(YY));
	moveSingleStage(ZZ, positionXYZ.at(ZZ));
}

bool Stage::isMoving(const Axis axis) const
{
	BOOL isMoving;

	if (!PI_IsMoving(mID.at(axis), mNstagesPerController, &isMoving))	//~55 ms to execute this function
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage " + axisToString(axis));

	return isMoving;
}

void Stage::waitForMotionToStopSingleStage(const Axis axis) const
{
	BOOL isMoving;
	do {
		if (!PI_IsMoving(mID.at(axis), mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + axisToString(axis));

		std::cout << ".";
	} while (isMoving);

	std::cout << "\n";
}

void Stage::waitForMotionToStopAllStages() const
{
	std::cout << "Stages moving to the new position: ";

	BOOL isMoving_x, isMoving_y, isMoving_z;
	do {
		if (!PI_IsMoving(mID.at(XX), mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mID.at(YY), mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mID.at(ZZ), mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << ".";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}

void Stage::stopAllstages() const
{
	PI_StopAll(mID.at(XX));
	PI_StopAll(mID.at(YY));
	PI_StopAll(mID.at(ZZ));

	std::cout << "Stages stopped\n";
}

//Request the velocity of the stage
double Stage::downloadSingleVelocity(const Axis axis) const
{
	double vel_mmps;
	if (!PI_qVEL(mID.at(axis), mNstagesPerController, &vel_mmps))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the velocity for the stage " + axisToString(axis));

	//std::cout << vel_mmps << " mm/s\n";
	return vel_mmps * mmps;					//Multiply by mmps to convert from explicit to implicit units
}

//Set the velocity of the stage
void Stage::setSingleVelocity(const Axis axis, const double vel) const
{
	if (vel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The velocity must be greater than zero for the stage " + axisToString(axis));

	const double vel_mmps = vel / mmps;		//Divide by mmps to convert implicit to explicit units
	if (!PI_VEL(mID.at(axis), mNstagesPerController, &vel_mmps))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the velocity for the stage " + axisToString(axis));
}

//Set the velocity of the stage 
void Stage::setAllVelocities(const double3 vel) const
{
	setSingleVelocity(XX, vel.at(XX));
	setSingleVelocity(YY, vel.at(YY));
	setSingleVelocity(ZZ, vel.at(ZZ));
}

//Each stage driver has 4 DO channels that can be used to monitor the stage position, motion, etc
//This function requests the trigger parameters of the stage. Consult the manual for C-863 p165
//1: trigger step in mm
//2: axis of the controller ( always 1 because each controller has only 1 stage)
//3: trigger mode (0: position distance, 2: on target, 6: in motion, 7: position+offset)
//7: polarity (0 for active low, 1 for active high)
//8: start threshold in mm
//9: stop threshold in mm
//10: trigger position in mm
double Stage::downloadDOtriggerSingleParam(const Axis axis, const int DOchan, const StageDOparam param) const
{
	const int triggerParam = static_cast<int>(param);
	double value;
	if (!PI_qCTO(mID.at(axis), &DOchan, &triggerParam, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger config for the stage " + axisToString(axis));

	//std::cout << value << "\n";
	return value;
}

void Stage::setDOtriggerSingleParam(const Axis axis, const int DOchan, const StageDOparam paramId, const double value) const
{
	const int triggerParam = static_cast<int>(paramId);
	if (!PI_CTO(mID.at(axis), &DOchan, &triggerParam, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger config for the stage " + axisToString(axis));
}

void Stage::setDOtriggerAllParams(const Axis axis, const int DOchan, const double triggerStep, const StageDOtriggerMode triggerMode, const double startThreshold, const double stopThreshold) const
{
	if ( triggerStep <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The trigger step must be greater than zero");

	if (startThreshold < mStagePosLimitXYZ.at(axis).at(0) || startThreshold > mStagePosLimitXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'startThreshold is out of bound for the stage " + axisToString(axis));


	setDOtriggerSingleParam(axis, DOchan, TriggerStep, triggerStep / mm);					//Trigger step
	setDOtriggerSingleParam(axis, DOchan, AxisNumber, 1);									//Axis of the controller (always 1 because each controller has only 1 stage)
	setDOtriggerSingleParam(axis, DOchan, TriggerMode, static_cast<double>(triggerMode));	//Trigger mode
	setDOtriggerSingleParam(axis, DOchan, Polarity, 1);										//Polarity (0 for active low, 1 for active high)
	setDOtriggerSingleParam(axis, DOchan, StartThreshold, startThreshold / mm);				//Start threshold
	setDOtriggerSingleParam(axis, DOchan, StopThreshold, stopThreshold / mm);				//Stop threshold
}

//Request the enable/disable status of the stage DO
bool Stage::isDOtriggerEnabled(const Axis axis, const int DOchan) const
{
	BOOL triggerState;
	if (!PI_qTRO(mID.at(axis), &DOchan, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger EN/DIS stage for the stage " + axisToString(axis));

	//std::cout << triggerState << "\n";
	return triggerState;
}

//Enable or disable the stage DO
void Stage::setDOtriggerEnabled(const Axis axis, const int DOchan, const BOOL triggerState) const
{
	if (!PI_TRO(mID.at(axis), &DOchan, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger EN/DIS state for the stage " + axisToString(axis));
}

//Each stage driver has 4 DO channels that can be used to monitor the stage position, motion, etc
//Print out the relevant parameters
void Stage::printStageConfig(const Axis axis, const int chan) const
{
	switch (axis)
	{
	case XX:
		//Only DO1 is wired to the FPGA
		if (chan != 1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	case YY:
		//Only DO1 is wired to the FPGA
		if (chan != 1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	case ZZ:
		//Only DO1 and DO2 are wired to the FPGA
		if (chan < 1 || chan > 2)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 and DO2 are currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	}

	const double triggerStep_mm = downloadDOtriggerSingleParam(axis, chan, TriggerStep);
	const int triggerMode = static_cast<int>(downloadDOtriggerSingleParam(axis, chan, TriggerMode));
	const int polarity = static_cast<int>(downloadDOtriggerSingleParam(axis, chan, Polarity));
	const double startThreshold_mm = downloadDOtriggerSingleParam(axis, chan, StartThreshold);
	const double stopThreshold_mm = downloadDOtriggerSingleParam(axis, chan, StopThreshold);
	const double triggerPosition_mm = downloadDOtriggerSingleParam(axis, chan, TriggerPosition);
	const bool triggerState = isDOtriggerEnabled(axis, chan);
	const double vel = downloadSingleVelocity(axis);

	std::cout << "Configuration for the stage = " << axisToString(axis) << ", DOchan = " << chan << ":\n";
	std::cout << "is DO trigger enabled? = " << triggerState << "\n";
	std::cout << "Trigger step = " << triggerStep_mm << " mm\n";
	std::cout << "Trigger mode = " << triggerMode << "\n";
	std::cout << "Polarity = " << polarity << "\n";
	std::cout << "Start threshold position = " << startThreshold_mm << " mm\n";
	std::cout << "Stop threshold position = " << stopThreshold_mm << " mm\n";
	std::cout << "Trigger position = " << triggerPosition_mm << " mm\n";
	std::cout << "Vel = " << vel / mmps << " mm/s\n\n";
}
#pragma endregion "Stages"





/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/
