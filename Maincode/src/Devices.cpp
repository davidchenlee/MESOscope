#include "Devices.h"

#pragma region "Image"
Image::Image(FPGAns::RTsequence &RTsequence) : mRTsequence(RTsequence), mTiff(mRTsequence.mWidthPerFrame_pix, mRTsequence.mHeightPerFrame_pix, mRTsequence.mNframes)
{
	mBufArrayA = new U32[mRTsequence.mNpixAllFrames];
	mBufArrayB = new U32[mRTsequence.mNpixAllFrames];
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

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
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, 0, timeout_ms, &nElemToReadA));
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, 0, timeout_ms, &nElemToReadB));
		//std::cout << "FIFOOUTpc cleanup A/B: " << nElemToReadA << "/" << nElemToReadB << std::endl;
		//getchar();

		if (nElemToReadA == 0 && nElemToReadB == 0)
			break;

		if (nElemToReadA > 0)
		{
			nElemToReadA = min(bufSize, nElemToReadA);	//Min between bufSize and nElemToReadA
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, nElemToReadA, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalA += nElemToReadA;
		}
		if (nElemToReadB > 0)
		{
			nElemToReadB = min(bufSize, nElemToReadB);	//Min between bufSize and nElemToReadB
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, nElemToReadB, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalB += nElemToReadB;
		}
	}
	if (nElemTotalA > 0 || nElemTotalB > 0)
		std::cout << "FIFOOUTpc garbage collector called. Number of elements cleaned up in FIFOOUTpc A/B: " << nElemTotalA << "/" << nElemTotalB << std::endl;
}

//Configure FIFOOUTpc. According to NI, this step is optional
void Image::configureFIFOOUTpc_(const U32 depth) const
{
	U32 actualDepth;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << std::endl;
}

//Stop the connection between FIFOOUTpc and FIFOOUTfpga
void Image::stopFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
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
	
	const int readFifoWaitingTime_ms = 5;			//Waiting time between each iteration
	const U32 timeout_ms = 100;						//FIFOOUTpc timeout
	
	//Null read timeout
	int timeout_iter = 100;						//Timeout the whileloop if the data download fails
	int nullReadCounterA = 0;
	int nullReadCounterB = 0;

	//FIFOOUT
	int nElemTotalA = 0; 					//Total number of elements read from FIFOOUTpc A
	int nElemTotalB = 0; 					//Total number of elements read from FIFOOUTpc B
	
	while (nElemTotalA < mRTsequence.mNpixAllFrames || nElemTotalB < mRTsequence.mNpixAllFrames)
	{
		Sleep(readFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time for max transfer bandwidth

		readChunk_(nElemTotalA, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, mBufArrayA, nullReadCounterA);	//FIFOOUTpc A
		readChunk_(nElemTotalB, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, mBufArrayB, nullReadCounterB);	//FIFOOUTpc B

		if (nullReadCounterA > timeout_iter && nullReadCounterB > timeout_iter)
			throw ImageException((std::string)__FUNCTION__ + ": FIFOOUTpc downloading timeout");

		//std::cout << "FIFO A: " << nElemTotalA << "\tFIFO B: " << nElemTotalB << std::endl;	//For debugging
		//std::cout << "nullReadCounter A: " << nullReadCounterA << "\tnullReadCounter: " << nullReadCounterB << std::endl;	//For debugging
	}

	/*
	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;
	std::cout << "FIFOOUT bandwidth: " << 2 * 32 * mRTsequence.mNpixAllFrames / duration / 1000 << " Mbps" << std::endl; //2 FIFOOUTs of 32 bits each
	std::cout << "Total of elements read: " << nElemTotalA << "\t" << nElemTotalB << std::endl; //Print out the total number of elements read
	*/
	

	//If all the expected data is NOT read successfully
	if (nElemTotalA <mRTsequence.mNpixAllFrames || nElemTotalB < mRTsequence.mNpixAllFrames)
		throw ImageException((std::string)__FUNCTION__ + ": Received less FIFOOUT elements than expected ");
}

//Read a chunk of data in the FIFOpc
void Image::readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, U32* buffer, int &nullReadCounter)
{
	U32 dummy;
	U32 nElemToRead = 0;				//Elements remaining in FIFOOUTpc
	const U32 timeout_ms = 100;			//FIFOOUTpc timeout

	if (nElemRead < mRTsequence.mNpixAllFrames)		//Skip if all the data have already been transferred
	{
		//By requesting 0 elements from FIFOOUTpc, the function returns the number of elements available. If no data is available, nElemToRead = 0 is returned
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), FIFOOUTpc, buffer, 0, timeout_ms, &nElemToRead));
		//std::cout << "Number of elements remaining in FIFOOUT: " << nElemToRead << std::endl;	//For debugging

		//If data available in FIFOOUTpc, retrieve it
		if (nElemToRead > 0)
		{
			//If more data than expected
			if (static_cast<int>(nElemRead + nElemToRead) > mRTsequence.mNpixAllFrames)
				throw std::runtime_error((std::string)__FUNCTION__ + ": FIFO buffer overflow");

			//Retrieve the elements in FIFOOUTpc
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTsequence.mFpga).getFpgaHandle(), FIFOOUTpc, buffer + nElemRead, nElemToRead, timeout_ms, &dummy));

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
	for (int lineIndex = 1; lineIndex < mRTsequence.mHeightAllFrames_pix; lineIndex += 2)
		std::reverse(mBufArrayB + lineIndex * mRTsequence.mWidthPerFrame_pix, mBufArrayB + lineIndex * mRTsequence.mWidthPerFrame_pix + mRTsequence.mWidthPerFrame_pix);
}

//When multiplexing later on, each U32 element in bufArray_B must be deMux in 8 segments of 4-bits each
void Image::demultiplex_()
{
	for (int pixIndex = 0; pixIndex < mRTsequence.mNpixAllFrames; pixIndex++)
	{
		unsigned char upscaled = mRTsequence.mUpscaleU8 * mBufArrayB[pixIndex]; //Upscale the buffer to go from 4-bit to 8-bit

		//If upscaled overflows
		if (upscaled > _UI8_MAX)
			upscaled = _UI8_MAX;

		//Transfer the result to Tiff
		(mTiff.accessTiff())[pixIndex] = upscaled;
	}
}

void Image::acquire()
{
	mRTsequence.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTsequence.uploadRT();			//Load the RT sequence in mVectorOfQueues to the FPGA
	startFIFOOUTpc_();				//Establish the connection between FIFOOUTfpga and FIFOOUTpc and cleans up any residual data from the previous run
	mRTsequence.triggerRT();		//Trigger the RT sequence. If triggered too early, FIFOOUTfpga will probably overflow

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
			std::cerr << "An ImageException has occurred in: " << e.what() << std::endl;
		}
	}
}

void Image::initialize()
{
	mRTsequence.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTsequence.uploadRT();			//Load the RT sequence in mVectorOfQueues to the FPGA
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
			std::cerr << "An ImageException has occurred in: " << e.what() << std::endl;
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
void Image::saveTiffSinglePage(std::string filename, const OverrideFileSelector overrideFlag) const
{
	mTiff.saveToFile(filename, SINGLEPAGE, overrideFlag);
}

//Save each frame in mTiff in a different Tiff page
void Image::saveTiffMultiPage(std::string filename, const OverrideFileSelector overrideFlag) const
{
	mTiff.saveToFile(filename, MULTIPAGE, overrideFlag);
}

//Access the Tiff data in the Image object
unsigned char* const Image::accessTiff() const
{
	return mTiff.accessTiff();
}
#pragma endregion "Image"


#pragma region "Vibratome"
Vibratome::Vibratome(const FPGAns::FPGA &fpga): mFpga(fpga){}

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::startStop_() const
{
	const int SleepTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTstart, true));

	Sleep(SleepTime);

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
}

//Move the head of the vibratome forward or backward for the duration 'duration_ms'. The timing varies in approx 1 ms
void Vibratome::moveHead_(const int duration_ms, const MotionDir channel) const
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const int minDuration_ms = 10;		//in ms
	const int delay_ms = 1;				//Used to roughly calibrate the pulse length

	switch (channel)
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

	if (duration_ms >= minDuration_ms)
		Sleep(duration_ms - delay_ms);
	else
	{
		Sleep(minDuration_ms - delay_ms);
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << minDuration_ms << "ms" << std::endl;
	}
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), selectedChannel, false));
}

void Vibratome::cutAndRetract(const int distance_mm) const
{
	const int cuttingTime_ms = static_cast<int>(1000.0 * distance_mm / mCuttingSpeed_mmps);
	const int retractingTime_ms = static_cast<int>(1000 * distance_mm / mMovingSpeed_mmps);

	startStop_();
	std::cout << "The vibratome is cutting for " << cuttingTime_ms/1000.0 << " seconds" << std::endl;
	Sleep(cuttingTime_ms);
	Sleep(2000);
	std::cout << "The vibratome is retracting for " << retractingTime_ms/1000.0 << " seconds" << std::endl;
	moveHead_(retractingTime_ms, BACKWARD);
}

void Vibratome::reset(const int distance_mm) const
{
	const int retractingTime_ms = static_cast<int>(1000 * distance_mm / mMovingSpeed_mmps);
	std::cout << "The vibratome is retracting for " << retractingTime_ms / 1000.0 << " seconds" << std::endl;
	moveHead_(retractingTime_ms, BACKWARD);
}


#pragma endregion "Vibratome"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const FPGAns::RTsequence &RTsequence): mRTsequence(RTsequence)
{	
	//Calculate the spatial fill factor
	const double temporalFillFactor = mRTsequence.mWidthPerFrame_pix * mRTsequence.mDwell_us / halfPeriodLineclock_us;
	if (temporalFillFactor > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");
	else
		mFillFactor = sin(PI / 2 * temporalFillFactor);					//Note that the fill factor doesn't depend on the RS amplitude because the RS period is fixed

	//std::cout << "Fill factor = " << mFillFactor << std::endl;		//For debugging

	//Download the current control voltage from the FPGA and update the scan parameters
	mControl_V = downloadControl_V();									//Control voltage
	mFullScan_um = mControl_V / mVoltPerUm;								//Full scan FOV = distance from turning point to turning point
	mFFOV_um = mFullScan_um * mFillFactor;								//FFOV
	mSampRes_umPerPix = mFFOV_um / mRTsequence.mWidthPerFrame_pix;		//Spatial sampling resolution
}

//Set the control voltage that determines the scanning amplitude
void ResonantScanner::setVoltage_(const double control_V)
{
	if (control_V < 0 || control_V > mVMAX_V)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage must be in the range 0-" + std::to_string(mVMAX_V) + " V" );

	//Update the scan parameters
	mControl_V = control_V;												//Control voltage
	mFullScan_um = control_V / mVoltPerUm;								//Full scan FOV
	mFFOV_um = mFullScan_um * mFillFactor;								//FFOV
	mSampRes_umPerPix = mFFOV_um / mRTsequence.mWidthPerFrame_pix;		//Spatial sampling resolution

	//Upload the control voltage
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::convertVoltToI16(mControl_V)));
}

//Set the full FOV of the microscope. FFOV does not include the cropped out areas at the turning points
void ResonantScanner::setFFOV(const double FFOV_um)
{
	//Update the scan parameters
	mFullScan_um = FFOV_um / mFillFactor;								//Full scan FOV
	mControl_V = mFullScan_um * mVoltPerUm;								//Control voltage
	mFFOV_um = FFOV_um;													//FFOV
	mSampRes_umPerPix = mFFOV_um / mRTsequence.mWidthPerFrame_pix;		//Spatial sampling resolution
	//std::cout << "mControl_V = " << mControl_V << std::endl; //For debugging

	if (mControl_V < 0 || mControl_V > mVMAX_V)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested FFOV must be in the range 0-" + std::to_string(mVMAX_V/mVoltPerUm) + " um");

	//Upload the control voltage
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::convertVoltToI16(mControl_V)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn_um(const double FFOV_um)
{
	setFFOV(FFOV_um);
	Sleep(mDelay_ms);
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV_um << " um" << std::endl;
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOn_V(const double control_V)
{
	setVoltage_(control_V);
	Sleep(mDelay_ms);
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << control_V << " V" << std::endl;
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(mDelay_ms);
	setVoltage_(0);
	std::cout << "RS successfully turned off" << std::endl;
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControl_V()
{
	I16 control_I16;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadI16((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

	return FPGAns::convertI16toVolt(control_I16);
}

//Spatial sampling resolution (um per pixel)
double ResonantScanner::getSamplingResolution_um()
{
	return mSampRes_umPerPix;
}

//Check if the RS is set to run. It does not actually check if the RS is running, for example, by looking at the RSsync signal
void ResonantScanner::isRunning()
{
	//Retrieve the state of the RS from the FPGA (see the LabView implementation)
	NiFpga_Bool isRunning;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadBool((mRTsequence.mFpga).getFpgaHandle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));

	char input_char;
	while (!isRunning)
	{
		std::cout << "RS seems OFF. Input 0 to exit or any other key to try again ";
		std::cin >> input_char;

		if (input_char == '0')
			throw std::runtime_error((std::string)__FUNCTION__ + ": Sequence terminated");
	}
}

#pragma endregion "Resonant scanner"


#pragma region "Galvo"

Galvo::Galvo(FPGAns::RTsequence &RTsequence, const RTchannel galvoChannel): mRTsequence(RTsequence), mGalvoRTchannel(galvoChannel)
{
	if ( mGalvoRTchannel != GALVO1 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
}

void Galvo::pushVoltageSinglet(const double timeStep, const double AO_V) const
{
	mRTsequence.pushAnalogSinglet(mGalvoRTchannel, timeStep, AO_V);
}

void Galvo::voltageLinearRamp(const double timeStep, const double rampLength, const double Vi_V, const double Vf_V) const
{
	mRTsequence.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, Vi_V, Vf_V);
}

void Galvo::positionLinearRamp(const double timeStep, const double rampLength, const double xi_V, const double xf_V) const
{
	mRTsequence.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, voltPerUm * xi_V, voltPerUm * xf_V);
}

void Galvo::voltageToZero() const
{
	mRTsequence.pushAnalogSinglet(mGalvoRTchannel, AO_tMIN_us, 0 * V);
}
#pragma endregion "Galvo"


#pragma region "PMT16X"
PMT16X::PMT16X()
{
	mSerial = new serial::Serial(mPort, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
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
		std::cout << "Warning in " + (std::string)__FUNCTION__  + ": CheckSum mismatch" << std::endl;
	
	//Print out the gains
	std::cout << "PMT16X gains:" << std::endl;
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << (int)parameters.at(ii) << std::endl;		
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
		std::cout << "PMT16X channel " << channel << " successfully set to " << gain << std::endl;
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch" << std::endl;
}

void PMT16X::setAllGainToZero() const
{
	std::vector<uint8_t> parameters = sendCommand_({ 'R' }); //The manual says that this sets all the gains to 255, but it really does it to 0
															//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. The second char returned is the sumcheck
	if (parameters.at(0) == 'R' && parameters.at(1) == 'R')
		std::cout << "All PMT16X gains successfully set to 0" << std::endl;
}

void PMT16X::setAllGain(const int gain) const
{
	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain must be in the range 0-255");

	std::vector<uint8_t> parameters = sendCommand_({ 'S', (uint8_t)gain });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'S' && parameters.at(1) == (uint8_t)gain && parameters.at(2) == sumCheck_(parameters, parameters.size() - 2))
		std::cout << "All PMT16X gains successfully set to " << gain << std::endl;
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch" << std::endl;
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
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch" << std::endl;

	//Print out the gains
	std::cout << "PMT16X gains successfully set to:" << std::endl;
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << (int)parameters.at(ii) << std::endl;

}

void PMT16X::readTemp() const
{
	std::vector<uint8_t> parameters = sendCommand_({ 'T' });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'T' || parameters.at(4) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch" << std::endl;

	const int TEMPH = (int)(parameters.at(1));
	const int TEMPL = (int)(parameters.at(2));
	const double temp_C = TEMPH + 0.01 * TEMPL; //According to the manual

	const int alertTemp_C = (int)(parameters.at(3));

	std::cout << "PMT16X temperature = " << temp_C << " \370C" << std::endl;
	std::cout << "PMT16X alert temperature = " << alertTemp_C <<  " \370C" << std::endl;
}
#pragma endregion "PMT16X"


#pragma region "Filterwheel"
Filterwheel::Filterwheel(const FilterwheelID ID): mDeviceID(ID)
{
	switch (ID)
	{
	case FWDET:
		mPort = assignCOM.at(COMFWDET);
		mDeviceName = "detection filterwheel";
		break;
	case FWEXC:
		mPort = assignCOM.at(COMFWEXC);
		mDeviceName = "excitation filterwheel";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected filterwheel unavailable");
	}

	try
	{
		mSerial = new serial::Serial(mPort, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
		downloadColor_();	//Download the current filter position
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mDeviceName);
	}
}

Filterwheel::~Filterwheel()
{
	mSerial->close();
}

//Convert from enum Filtercolor to string
std::string Filterwheel::convertToString_(const Filtercolor color) const
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
	default:
		colorStr = "UNKNOWN";
	}
	return colorStr;
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
		//std::cout << "Full RxBuffer: " << RxBuffer << std::endl; //For debugging

		//Delete echoed command
		std::string::size_type ii = RxBuffer.find(TxBuffer);
		if (ii != std::string::npos)
			RxBuffer.erase(ii, TxBuffer.length());

		//Delete CR and >
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
		//RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

		//std::cout << "Cleaned RxBuffer: " << RxBuffer << std::endl; //For debugging
		mColor = static_cast<Filtercolor>(std::stoi(RxBuffer));	//convert string to int, and then to Filtercolor
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mDeviceName);
	}
}

void Filterwheel::setColor(const Filtercolor color)
{
	if (color != mColor)
	{
		std::string TxBuffer("pos=" + std::to_string(color) + "\r");
		std::string RxBuffer;

		try
		{
			mSerial->write(TxBuffer);

			//Find the shortest way to reach the targeted position
			const int minPos = (std::min)(color, mColor);
			const int maxPos = (std::max)(color, mColor);
			const int diffPos = maxPos - minPos;
			const int minSteps = (std::min)(diffPos, mNpos - diffPos);

			//std::cout << "Tuning the " << mDeviceName << " to " + convertToString_(color) << std::endl;
			Sleep((int)(1000.0 * minSteps / mTuningSpeed_Hz)); //Wait until the filterwheel stops turning the turret

			mSerial->read(RxBuffer, mRxBufSize);		//Read RxBuffer to flush it. Serial::flush() doesn't work
														//std::cout << "setColor full RxBuffer: " << RxBuffer << std::endl; //For debugging

			downloadColor_();
			if (color == mColor)
				std::cout << "The " << mDeviceName << " successfully set to " + convertToString_(mColor) << std::endl;
			else
				std::cout << "WARNING: The " << mDeviceName << " might not be in the correct position " + convertToString_(color) << std::endl;
		}
		catch (const serial::IOException)
		{
			throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mDeviceName);
		}
	}
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

	setColor(color);
}
#pragma endregion "Filterwheel"

#pragma region "Laser"
Laser::Laser(RTchannel laserID): mLaserID(laserID)
{
	switch (mLaserID)
	{
	case VISION:
		mLaserNameString = "VISION";
		mPort = assignCOM.at(COMVISION);
		mBaud = 19200;
		break;
	case FIDELITY:
		mLaserNameString = "FIDELITY";
		mPort = assignCOM.at(COMFIDELITY);
		mBaud = 115200;
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial = new serial::Serial(mPort, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure establishing serial communication with " + mLaserNameString);
	}

	mWavelength_nm = downloadWavelength_();
}

Laser::~Laser()
{
	mSerial->close();
}

int Laser::downloadWavelength_()
{
	switch (mLaserID)
	{
	case VISION:
		try
		{
			const std::string TxBuffer("?VW");		//Command to the laser
			std::string RxBuffer;					//Reply from the laser
			mSerial->write(TxBuffer + "\r");
			mSerial->read(RxBuffer, mRxBufSize);

			//Delete echoed command. Echoing could be disabled on the laser but deleting it is safer and more general
			std::string keyword("?VW ");
			std::string::size_type i = RxBuffer.find(keyword);
			if (i != std::string::npos)
				RxBuffer.erase(i, keyword.length());

			//Delete "CHAMELEON>". This frase could be disabled on the laser, but deleting it is safer and more general
			keyword = "CHAMELEON>";
			i = RxBuffer.find(keyword);
			if (i != std::string::npos)
				RxBuffer.erase(i, keyword.length());

			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());
			//std::cout << RxBuffer << std::endl;	//For debugging

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
	std::cout << mLaserNameString +  " wavelength is " << mWavelength_nm << " nm" << std::endl;
}

void Laser::setWavelength(const int wavelength_nm)
{
	switch (mLaserID)
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

				//std::cout << "Sleep time in ms: " << (int) std::abs(1000.0*(mWavelength_nm - wavelength_nm) / mTuningSpeed_nm_s) << std::endl;	//For debugging
				std::cout << "Tuning VISION to " << wavelength_nm << " nm" << std::endl;
				Sleep((int)std::abs(1000.0*(mWavelength_nm - wavelength_nm) / mTuningSpeed_nm_s));	//Wait till the laser finishes tuning

				mSerial->read(RxBuffer, mRxBufSize);	//Read RxBuffer to flush it. Serial::flush() doesn't work. The message reads "CHAMELEON>"

				downloadWavelength_();
				if (mWavelength_nm = wavelength_nm)
					std::cout << "VISION wavelength successfully set to " << wavelength_nm << " nm" << std::endl;
				else
					std::cout << "WARNING: VISION might not be in the correct wavelength " << wavelength_nm << " nm" << std::endl;
			}
			catch (const serial::IOException)
			{
				throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
			}
		}
		break;
	case FIDELITY:
		std::cout << "WARNING: FIDELITY wavelength only supports 1040 nm" << std::endl;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

//Open or close the internal shutter of the laser
void Laser::setShutter(const bool state) const
{
	std::string TxBuffer;		//Command to the laser
	std::string RxBuffer;		//Reply from the laser

	switch (mLaserID)
	{
	case VISION:
		TxBuffer = "S=" + std::to_string(state);
		break;
	case FIDELITY:
		TxBuffer = "SHUTTER=" + std::to_string(state);		//Command to the laser
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial->write(TxBuffer + "\r");
		mSerial->read(RxBuffer, mRxBufSize);	//Read RxBuffer to flush it. Serial::flush() doesn't work.

		if (state)
			std::cout << mLaserNameString + " shutter successfully opened" << std::endl;
		else
			std::cout << mLaserNameString + " shutter successfully closed" << std::endl;
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with " + mLaserNameString);
	}
}
#pragma endregion "Laser"


#pragma region "Shutters"
Shutter::Shutter(const FPGAns::FPGA &fpga, RTchannel laserID) : mFpga(fpga)
{
	switch (laserID)
	{
	case VISION:
		mDeviceID = NiFpga_FPGAvi_ControlBool_ShutterVision;
		break;
	case FIDELITY:
		mDeviceID = NiFpga_FPGAvi_ControlBool_ShutterFidelity;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected shutter unavailable");
	}
}

Shutter::~Shutter()
{
	//This is to prevent keeping the shutter open in case of an exception
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mDeviceID, false));
}

void Shutter::openClose(const bool state) const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mDeviceID, state));
}


void Shutter::pulseHigh() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mDeviceID, true));

	Sleep(mDelay_ms);

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), mDeviceID, false));
}
#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Curently, output of the pockels cell is hardcoded on the FPGA side.  The pockels' output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(FPGAns::RTsequence &RTsequence, const RTchannel laserID, const int wavelength_nm) : mRTsequence(RTsequence), mPockelsRTchannel(laserID), mWavelength_nm(wavelength_nm), mShutter(mRTsequence.mFpga, mPockelsRTchannel)
{
	if (mPockelsRTchannel != VISION && mPockelsRTchannel != FIDELITY)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels channel unavailable");

	switch (mPockelsRTchannel)
	{
	case VISION:
		mScalingRTchannel = SCALINGVISION;
		break;
	case FIDELITY:
		mScalingRTchannel = SCALINGFIDELITY;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	//Initialize all the scaling factors to 1.0. In LV, I could not sucessfully default the LUT to 0d16384 = 0b0100000000000000 = 1 for a fixed point Fx2.14
	for (int ii = 0; ii < mRTsequence.mNframes; ii++)
		mRTsequence.pushAnalogSingletFx2p14(mScalingRTchannel, 1.0);
}

double PockelsCell::convert_mWToVolt_(const double power_mW) const
{
	double a, b, c;		//Calibration parameters

	//VISION
	switch (mPockelsRTchannel)
	{
	case VISION:
		if (mWavelength_nm == 750) {
			a = 788;
			b = 0.6152;
			c = -0.027;
		}
		else if (mWavelength_nm == 940) {
			a = 464;
			b = 0.488;
			c = -0.087;
		}
		else if (mWavelength_nm == 1040) {
			a = 200;
			b = 0.441;
			c = 0.037;
		}
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		break;

		//FIDELITY
	case FIDELITY:
		a = 101.20;
		b = 0.276;
		c = -0.049;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	double arg = sqrt(power_mW / a);
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Arg of asin is greater than 1");

	switch (mPockelsRTchannel)
	{
	case VISION:
		return asin(arg) / b + c;
	case FIDELITY:
		//return (PI - asin(arg)) / b + c; //different expression from VISION because currently no HWP in front of the pockels
		return asin(arg) / b + c;
	default:
		return 0;
	}
}


void PockelsCell::pushVoltageSinglet(const double timeStep, const double AO_V) const
{
	if (AO_V < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTsequence.pushAnalogSinglet(mPockelsRTchannel, timeStep, AO_V);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P_mW, const OverrideFileSelector overrideFlag) const
{
	if (P_mW < 0 || P_mW > maxPower_mW)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's laser power must be in the range 0-" + std::to_string(P_mW));

	mRTsequence.pushAnalogSinglet(mPockelsRTchannel, timeStep, convert_mWToVolt_(P_mW), overrideFlag);
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void PockelsCell::voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const
{
	if (Vi_V < 0 || Vf_V < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTsequence.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, Vi_V, Vf_V);
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void  PockelsCell::powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const
{
	if (Pi_mW < 0 || Pf_mW < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTsequence.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, convert_mWToVolt_(Pi_mW), convert_mWToVolt_(Pf_mW));
}

void PockelsCell::voltageToZero() const
{
	mRTsequence.pushAnalogSinglet(mPockelsRTchannel, AO_tMIN_us, 0 * V);
}

//Linearly scale the pockels output across all the frames
void PockelsCell::scalingLinearRamp(const double Si, const double Sf) const
{
	if (Si < 0 || Sf < 0 || Si > 4 || Sf > 4)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested scaling factor must be in the range 0-4");

	if (mRTsequence.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	mRTsequence.clearQueue(mScalingRTchannel);	//Delete the default scaling factors of 1.0s created in the PockelsCell constructor

	for (int ii = 0; ii < mRTsequence.mNframes; ii++)
		mRTsequence.pushAnalogSingletFx2p14(mScalingRTchannel, Si + (Sf - Si) / (mRTsequence.mNframes - 1) * ii);
}

void PockelsCell::setShutter(const bool state) const
{
	mShutter.openClose(state);
}

#pragma endregion "Pockels cells"


//Integrate the lasers, pockels cells, and filterwheels in a single class
#pragma region "VirtualLaser"
VirtualLaser::VirtualLaser(FPGAns::RTsequence &RTsequence, const int wavelength_nm, const double power_mW): mWavelength_nm(wavelength_nm),
																											mVision(VISION), mFidelity(FIDELITY),
																											mPockelsVision(RTsequence, VISION, wavelength_nm), mPockelsFidelity(RTsequence, FIDELITY, 1040),
																											mFWexcitation(FWEXC), mFWdetection(FWDET)
{
	setWavelength(mWavelength_nm);
}

void VirtualLaser::setWavelength(const int wavelength_nm)
{
	//Use VISION for everything below 1040 nm
	if (wavelength_nm < 1040)
	{
		mLaserID = VISION;
		mVision.setWavelength(wavelength_nm);
	}
	//Use FIDELITY for 1040 nm
	else if (wavelength_nm == 1040)
	{
		mLaserID = FIDELITY;
	}
	else
	{
		throw std::invalid_argument((std::string)__FUNCTION__ + ": wavelength > 1040 nm not implemented");
	}

	mFWexcitation.setColor(wavelength_nm);
	mFWdetection.setColor(wavelength_nm);

	mWavelength_nm = wavelength_nm;
}

void VirtualLaser::pushPowerSinglet(const double timeStep, const double P_mW, const OverrideFileSelector overrideFlag) const
{
	switch (mLaserID)
	{
	case VISION:
		mPockelsVision.pushPowerSinglet(timeStep, P_mW, overrideFlag);
		break;
	case FIDELITY:
		mPockelsFidelity.pushPowerSinglet(timeStep, P_mW, overrideFlag);
		break;
	}
}

void VirtualLaser::setShutter(const bool state) const
{
	switch (mLaserID)
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
Stage::Stage()
{
	const std::string stageIDx = "116049107";	//X-stage (V-551.4B)
	const std::string stageIDy = "116049105";	//Y-stage (V-551.2B)
	const std::string stageIDz = "0165500631";	//Z-stage (ES-100)

	//Open the connections to the stage controllers and assign the IDs
	mID[XX] = PI_ConnectUSB(stageIDx.c_str());
	mID[YY] = PI_ConnectUSB(stageIDy.c_str());
	mID[ZZ] = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID[ZZ] = PI_ConnectUSB(stageIDz.c_str());

	if (mID[XX] < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");

	if (mID[YY] < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");

	if (mID[ZZ] < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	std::cout << "Connection to the stages successfully established\n";

	//Record the current position
	mPosition3_mm[XX] = downloadPosition_mm(XX);
	mPosition3_mm[YY] = downloadPosition_mm(YY);
	mPosition3_mm[ZZ] = downloadPosition_mm(ZZ);
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mID[XX]);
	PI_CloseConnection(mID[YY]);
	PI_CloseConnection(mID[ZZ]);
	std::cout << "Connection to the stages successfully closed\n";
}

//Recall the current position for the 3 stages
double3 Stage::readPosition3_mm() const
{
	return mPosition3_mm;
}

void Stage::printPosition3() const
{
	std::cout << "Stage X position = " << mPosition3_mm[XX] << " mm" << std::endl;
	std::cout << "Stage Y position = " << mPosition3_mm[YY] << " mm" << std::endl;
	std::cout << "Stage Z position = " << mPosition3_mm[ZZ] << " mm" << std::endl;
}

//Retrieve the stage position from the controller
double Stage::downloadPosition_mm(const Axis axis)
{
	double position_mm;
	if (!PI_qPOS(mID[axis], mNstagesPerController, &position_mm))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query position for the stage " + std::to_string(axis));

	return position_mm;
}

//Move the stage to the requested position
void Stage::moveStage(const Axis axis, const double position_mm)
{
	//Check if the requested position is within range
	if (position_mm < mPosMin3_mm[axis] || position_mm > mPosMax3_mm[axis])
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested position out of bounds for stage " + std::to_string(axis));

	//Move the stage
	if (mPosition3_mm[axis] != position_mm ) //Move only if the requested position is different from the current position
	{
		if (!PI_MOV(mID[axis], mNstagesPerController, &position_mm) )	//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage" + std::to_string(axis) + " to the target position");

		mPosition3_mm[axis] = position_mm;
	}
}

//Move the 3 stages to the requested position
void Stage::moveStage3(const double3 positionXYZ_mm)
{
	moveStage(XX, positionXYZ_mm[XX]);
	moveStage(YY, positionXYZ_mm[YY]);
	moveStage(ZZ, positionXYZ_mm[ZZ]);
}


bool Stage::isMoving(const Axis axis) const
{
	BOOL isMoving;

	if (!PI_IsMoving(mID[axis], mNstagesPerController, &isMoving))	//~55 ms to execute this function
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage " + std::to_string(axis));

	return isMoving;
}

void Stage::waitForMotionToStop(const Axis axis) const
{
	BOOL isMoving;
	do {
		if (!PI_IsMoving(mID[axis], mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + std::to_string(axis));

		std::cout << ".";
	} while (isMoving);

	std::cout << "\n";
}

void Stage::waitForMotionToStop3() const
{
	std::cout << "Updating the stage position: ";

	BOOL isMoving_x, isMoving_y, isMoving_z;
	do {
		if (!PI_IsMoving(mID[XX], mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mID[YY], mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mID[ZZ], mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << ".";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}

void Stage::stopALL() const
{
	PI_StopAll(mID[XX]);
	PI_StopAll(mID[YY]);
	PI_StopAll(mID[ZZ]);
}

//Request the configuration of the digital output
double Stage::qCTO_(const Axis axis, const int chan, const int triggerParam) const
{
	double value;
	if (!PI_qCTO(mID[axis], &chan, &triggerParam, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query CTO for the stage" + std::to_string(axis));

	//std::cout << value << std::endl;
	return value;

}

//Request the enable/disable status of the digital output
bool Stage::qTRO_(const Axis axis, const int chan) const
{
	BOOL triggerState;
	if (!PI_qTRO(mID[axis], &chan, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query TRO for the stage" + std::to_string(axis));

	//std::cout << triggerState << std::endl;
	return triggerState;
}

//Set the enable/disable status of the digital output
void Stage::TRO_(const Axis axis, const int chan, const BOOL triggerState) const
{
	if (!PI_TRO(mID[axis], &chan, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set TRO for the stage" + std::to_string(axis));
}

//Request the velocity of the stage in mm/s
double Stage::qVEL(const Axis axis) const
{
	double vel_mmPerS;
	if(!PI_qVEL(mID[axis], mNstagesPerController, &vel_mmPerS))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the velocity for the stage" + std::to_string(axis));

	//std::cout << vel_mmPerS << std::endl;
	return vel_mmPerS;
}

//Set the velocity of the stage in mm/s
void Stage::VEL(const Axis axis, const double vel_mmPerS) const
{
	if(!PI_VEL(mID[axis], mNstagesPerController, &vel_mmPerS))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the velocity for the stage" + std::to_string(axis));
}

//Request the CTO (trigger) configuration of the stage
void Stage::downloadConfiguration(const Axis axis, const int chan) const
{
	if (chan < 1 || chan > 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested channel must be in the range 1-4" + std::to_string(axis));

	//1: trigger step
	//2: axis
	//3: trigger mode (0: position distance, 2: on target, 6: in motion)
	//7: polarity
	//8: start threshold
	//9: stop threshold
	//10: trigger position
	double triggerStep = qCTO_(axis, chan, 1);
	double triggerMode = qCTO_(axis, chan, 3);
	double polarity = qCTO_(axis, chan, 7);
	double startThreshold = qCTO_(axis, chan, 8);
	double stopThreshold = qCTO_(axis, chan, 9);
	double triggerPosition = qCTO_(axis, chan, 10);
	bool triggerState = qTRO_(axis, chan);
	double vel_mmPerS = qVEL(axis);

	std::cout << "triggerStep: " << triggerStep << std::endl;
	std::cout << "triggerMode: " << triggerMode << std::endl;
	std::cout << "polarity: " << polarity << std::endl;
	std::cout << "startThreshold: " << startThreshold << std::endl;
	std::cout << "stopThreshold: " << stopThreshold << std::endl;
	std::cout << "triggerPosition: " << triggerPosition << std::endl;
	std::cout << "triggerState: " << triggerState << std::endl;
	std::cout << "vel_mmPerS: " << vel_mmPerS << std::endl;
}

//convert from (slice number, plane number, tile number) ---> absolute position (x,y,z)
//this function considers the overlaps in x, y, and z
double3 Stage::readAbsolutePosition3_mm(const int nSlice, const int nPlane, const int3 nTileXY) const
{
	const double mm = 1;
	const double um = 0.001;

	double3 absPosition_mm {};
	double3 initialPosition_mm { 31.9*mm, 9.5*mm, 18.546*mm };
	double3 overlap_um { 20.*um, 20.*um, 30.*um };
	double3 FFOVxy_um { 200.*um, 200.*um };						//Full FOV

	double sliceThickness_um = 100 * um;
	double stepZ_um = 1;

	absPosition_mm.at(0) = initialPosition_mm.at(0) + nTileXY.at(0) * (FFOVxy_um.at(0) - overlap_um.at(0));
	absPosition_mm.at(1) = initialPosition_mm.at(1) + nTileXY.at(1) * (FFOVxy_um.at(1) - overlap_um.at(1));
	absPosition_mm.at(2) = initialPosition_mm.at(2) - nSlice * (sliceThickness_um - overlap_um.at(2)) - nPlane * stepZ_um;

	return absPosition_mm;
}
#pragma endregion "Stages"


#pragma region "Command"
Command::Command(Action action, const int sleep_ms ) : mAction(action), mSleep_ms(sleep_ms)
{
}

std::string Command::actionToString_(Action action)
{
	switch (action)
	{
	case CUTSLICE:
		return "CUTSLICE";
	case MOVSTAGE:
		return "MOVSTAGE";
	case ACQSTACK:
		return "ACQSTACK";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action unavailable");
	}
}

void Command::printCommand() {}

void Command::printHeader()
{
	std::cout << "Action\t\tSleep_ms\tStackCtr_mm\tWavlen_nm\tScanDirZ\tz_um min/max\tP_mW min/max" << std::endl;
}

CutSection::CutSection(const int sleep_ms) : Command(CUTSLICE, sleep_ms)
{
}

void CutSection::printCommand()
{
	std::cout << actionToString_(mAction) << "\t" << mSleep_ms << std::endl;
}

AcqStack::AcqStack(const int sleep_ms, const double2 stackCenter_mm, const int wavelength_nm, const int scanDirZ, const double2 Z_um, const double2 P_mW) : 
	Command(ACQSTACK, sleep_ms), mStackCenter_mm(stackCenter_mm), mWavelength_nm(wavelength_nm), mScanDirZ(scanDirZ), mZ_um(Z_um), mP_mW(P_mW)
{
}

void AcqStack::printCommand()
{
	std::cout << actionToString_(mAction) << "\t" << mSleep_ms << "\t\t(" << mStackCenter_mm.at(XX) << "," << mStackCenter_mm.at(YY) << ")\t\t" << mWavelength_nm <<
		"\t\t" << mScanDirZ << "\t\t" << mZ_um.at(0) << "/" << mZ_um.at(1) << "\t\t" << mP_mW.at(0) << "/" << mP_mW.at(1) << "\t" << std::endl;
}

#pragma endregion "Command"

#pragma region "Sequencer"
Sequencer::Sequencer(const ROI roi_mm): mROI_mm(roi_mm)
{
	//Convert the ROI = (xmin, ymax, xmax, ymin) to the equivalent sample size
	mSampleSize_um.at(XX) = 1000 * (mROI_mm.at(2) - mROI_mm.at(0));
	mSampleSize_um.at(YY) = 1000 * (mROI_mm.at(1) - mROI_mm.at(3));

	if (mSampleSize_um.at(XX) < 0 || mSampleSize_um.at(YY) < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

	//Calculate the number of tiles
	mNtiles.at(XX) = static_cast<int>(std::ceil(mSampleSize_um.at(XX) / mFOV_um.at(XX)));		//Number of tiles in x
	mNtiles.at(YY) = static_cast<int>(std::ceil(mSampleSize_um.at(YY) / mFOV_um.at(YY)));		//Number of tiles in y
	mNtilesTotal = mNtiles.at(XX) * mNtiles.at(YY);												//Total number of tiles

	std::cout << "Ntiles x = " << mNtiles.at(XX) << "\tNtiles y = " << mNtiles.at(YY) << std::endl;
}

Sequencer::~Sequencer()
{
	for (int iter = 0; iter < static_cast<int>(mCommandList.size()); iter++)
		delete mCommandList.at(iter);
}

void Sequencer::pushCommand(Command *command)
{
	mCommandList.push_back(command);
}


void Sequencer::printCommandList()
{
	//Print out the commandline labels
	if (!mCommandList.empty())
	{
		std::cout << "#\t";
		mCommandList.at(0)->printHeader();
	}

	for (int iter = 0; iter < static_cast<int>(mCommandList.size()); iter++)
	{
		std::cout << iter << "\t";
		mCommandList.at(iter)->printCommand();
	}
}


//The idea is to input the iteration number iter = 0, 1, ..., mNtilesTotal and output the corresponding tile index in the 2D matrix of mNtiles.at(XX)
//by mNtiles.at(YY). The tile assignement depends on the scanning strategy.
int2 Sequencer::snakeIndices(const int iter, const InitialStagePosition initialStagePosition) const
{
	if (iter < 0 || iter >= mNtilesTotal)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": index out of bound");

	int2 sequencedIndex;
	sequencedIndex.at(XX) = iter % (mNtiles.at(XX));										//x index
	sequencedIndex.at(YY) = static_cast<int>(std::floor(1.* iter / (mNtiles.at(XX))) );		//y index

	int stageMotionDirX, stageMotionDirY;

	//The initial position of the stage in the sample plane determines the scanning direction
	//The y-stage is the slowest to react because it sits under of the x and z stages.
	//Therefore, for a snake pattern, alternate the motion in x but keep the motion in y unchanged.
	switch (initialStagePosition)
	{
	case BOTTOMLEFT:
		stageMotionDirX = 1 - 2 * (sequencedIndex.at(YY) % 2); //1 for even y tile, -1 for odd y tile
		stageMotionDirY = 1;
		break;
	case TOPLEFT:
		stageMotionDirX = 1 - 2 * (sequencedIndex.at(YY) % 2); //1 for even y tile, -1 for odd y tile
		stageMotionDirY = -1;
		break;
	case TOPRIGHT:
		stageMotionDirX = -1 + 2 * (sequencedIndex.at(YY) % 2); //-1 for even y tile, 1 for odd y tile
		stageMotionDirY = -1;
		break;
	case BOTTOMRIGHT:
		stageMotionDirX = -1 + 2 * (sequencedIndex.at(YY) % 2); //-1 for even y tile, 1 for odd y tile
		stageMotionDirY = 1;
		break;
	}

	//Reverse the indexing order if the scanning direction is negative
	if (stageMotionDirX > 0 && stageMotionDirY > 0)
	{
		//No change
	}
	else if (stageMotionDirX > 0 && stageMotionDirY < 0)
	{
		sequencedIndex.at(YY) = mNtiles.at(YY) - 1 - sequencedIndex.at(YY);
	}
	else if (stageMotionDirX < 0 && stageMotionDirY > 0)
	{
		sequencedIndex.at(XX) = mNtiles.at(XX) - 1 - sequencedIndex.at(XX);
	}
	else if (stageMotionDirX < 0 && stageMotionDirY < 0)
	{
		sequencedIndex.at(XX) = mNtiles.at(XX) - 1 - sequencedIndex.at(XX);
		sequencedIndex.at(YY) = mNtiles.at(YY) - 1 - sequencedIndex.at(YY);
	}
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid direction of motion for the stage");

	return sequencedIndex;
}

double2 Sequencer::convertIndexToPosition_mm(const int2 tileIndices) const
{
	double2 stagePosition_mm;
	stagePosition_mm.at(XX) = mFOV_um.at(XX)/1000 * (tileIndices.at(XX) + 0.5);	// (tileIndices + 0.5) ranges from 0.5 to (mNtiles - 0.5)
	stagePosition_mm.at(YY) = mFOV_um.at(YY)/1000 * (tileIndices.at(YY) + 0.5);	// (tileIndices + 0.5) ranges from 0.5 to (mNtiles - 0.5)

	return stagePosition_mm;
}



//To optimize the stage scanning, scan for 750 nm then scan back for 940 nm, etc
InitialStagePosition Sequencer::stageScanningDir(const int wavelength_nm)
{
	switch (wavelength_nm)
	{
	case 750:
		return BOTTOMLEFT;			//start scanning from the bottom-left
	case 940:
		if (mNtiles.at(YY) % 2)
			return TOPLEFT;		//start scanning from the top-left
		else
			return TOPRIGHT;	//start scanning from the top-right
	case 1040:
		return BOTTOMLEFT;			//start scanning from the bottom-left
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": ");
	}
}
#pragma endregion "sequencer"



/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/
