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

//Flush the residual data in FIFOOUTpc from the previous run, if any
void Image::FIFOOUTpcGarbageCollector_() const
{
	const U32 timeout_ms{ 100 };
	const int bufSize{ 10000 };

	U32 dummy;
	U32* garbage = new U32[bufSize];
	U32 nElemToReadA{ 0 }, nElemToReadB{ 0 };			//Elements to read from FIFOOUTpc A and B
	int nElemTotalA{ 0 }, nElemTotalB{ 0 }; 			//Total number of elements read from FIFOOUTpc A and B
	while (true)
	{
		//Check if there are elements in FIFOOUTpc
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, 0, timeout_ms, &nElemToReadA));
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, 0, timeout_ms, &nElemToReadB));
		//std::cout << "FIFOOUTpc cleanup A/B: " << nElemToReadA << "/" << nElemToReadB << "\n";
		//getchar();

		if (nElemToReadA == 0 && nElemToReadB == 0)
			break;

		if (nElemToReadA > 0)
		{
			nElemToReadA = min(bufSize, nElemToReadA);	//Min between bufSize and nElemToReadA
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, nElemToReadA, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalA += nElemToReadA;
		}
		if (nElemToReadB > 0)
		{
			nElemToReadB = min(bufSize, nElemToReadB);	//Min between bufSize and nElemToReadB
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, nElemToReadB, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalB += nElemToReadB;
		}
	}
	if (nElemTotalA > 0 || nElemTotalB > 0)
		std::cout << "FIFOOUTpc garbage collector called. Number of elements cleaned up in FIFOOUTpc A/B: " << nElemTotalA << "/" << nElemTotalB << "\n";
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
	
	const int readFifoWaitingTime_ms{ 5 };				//Waiting time between each iteration
	int timeout_iter{ 200 };							//Timeout the whileloop if the data transfer fails
	int nullReadCounterA{ 0 }, nullReadCounterB{ 0 };	//Null reading counters

	//FIFOOUT
	int nElemTotalA{ 0 }; 					//Total number of elements read from FIFOOUTpc A
	int nElemTotalB{ 0 }; 					//Total number of elements read from FIFOOUTpc B
	
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
	U32 nElemToRead{ 0 };				//Elements remaining in FIFOOUTpc
	const U32 timeout_ms{ 100 };		//FIFOOUTpc timeout

	if (nElemRead < mRTcontrol.mNpixAllFrames)		//Skip if all the data have already been transferred
	{
		//By requesting 0 elements from FIFOOUTpc, the function returns the number of elements available. If no data is available, nElemToRead = 0 is returned
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), FIFOOUTpc, buffer, 0, timeout_ms, &nElemToRead));
		//std::cout << "Number of elements remaining in FIFOOUT: " << nElemToRead << "\n";	//For debugging

		//If data available in FIFOOUTpc, retrieve it
		if (nElemToRead > 0)
		{
			//If more data than expected
			if (static_cast<int>(nElemRead + nElemToRead) > mRTcontrol.mNpixAllFrames)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Received more FIFO elements than expected");

			//Retrieve the elements in FIFOOUTpc
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32((mRTcontrol.mFpga).getHandle(), FIFOOUTpc, buffer + nElemRead, nElemToRead, timeout_ms, &dummy));

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
	{
		std::reverse(mBufArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix, mBufArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix + mRTcontrol.mWidthPerFrame_pix);
		std::reverse(mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix, mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix + mRTcontrol.mWidthPerFrame_pix);
	}
}

//Once multiplexing is implemented, each U32 element in bufArray_B must be deMux in 8 segments of 4-bits each
void Image::demultiplex_()
{
	//Define masks
	const U32 mask1 = 0x0000000F;

	for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixAllFrames; pixIndex++)
	{
		//TODO: pick up the corresponding 4-bit segment from mBufArrayA and mBufArrayB
		//Upscale the buffer to go from 4-bit to 8-bit
		U8 upscaled{ static_cast<U8>(mRTcontrol.mUpscaleFactorU8 * mBufArrayA[pixIndex]) };

		//If upscaled overflows
		if (upscaled > _UI8_MAX)
			upscaled = _UI8_MAX;

		//Transfer the result to a Tiff
		//For MUX16, resconstruct the image by assigning the stripes in the image
		(mTiff.pointerToTiff())[pixIndex] = upscaled;
	}
}

//Establish a connection between FIFOOUTpc and FIFOOUTfpga and. Optional according to NI
void Image::startFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}

//Configure FIFOOUTpc. Optional according to NI
void Image::configureFIFOOUTpc_(const U32 depth) const
{
	U32 actualDepth;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << "\n";
}

//Stop the connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI
void Image::stopFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
	//std::cout << "stopFIFO called\n";
}

void Image::acquire(const FIFOOUTenableSelector FIFOOUTenable)
{
	//Enable pushing data to FIFOOUTfpga. Disable for debugging
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTgateEnable, FIFOOUTenable));	

	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to the FPGA
	startFIFOOUTpc_();				//Establish connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI, but if not called, sometimes garbage is generated
	FIFOOUTpcGarbageCollector_();	//Clean up any residual data from a previous run
	mRTcontrol.triggerRT();			//Trigger the RT control. If triggered too early, FIFOOUTfpga will probably overflow

	if (FIFOOUTenable)
	{
		try
		{
			readFIFOOUTpc_();			//Read the data received in FIFOOUTpc
			correctInterleaved_();
			demultiplex_();				//Move the chuncks of data to the buffer array
			mTiff.mirrorOddFrames();	//The galvo (vectical axis of the image) performs bi-directional scanning. Divide the long vertical image in nFrames and vertically mirror the odd frames
		}
		catch (const ImageException &e) //Notify the exception and continue with the next iteration
		{
			std::cerr << "An ImageException has occurred in: " << e.what() << "\n";
		}
	}
}

void Image::initialize() const
{
	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to the FPGA
	startFIFOOUTpc_();				//Establish connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI, but if not called, sometimes garbage is generated
	FIFOOUTpcGarbageCollector_();	//Cleans up any residual data from the previous run
}

void Image::downloadData(const FIFOOUTenableSelector FIFOOUTenable)
{
	//Enable pushing data to FIFOOUTfpga. Disable for debugging
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTgateEnable, FIFOOUTenable));

	if (FIFOOUTenable)
	{
		try
		{
			readFIFOOUTpc_();			//Read the data received in FIFOOUTpc
		}
		catch (const ImageException &e) //Notify the exception and continue with the next iteration
		{
			std::cerr << "An ImageException has occurred in: " << e.what() << "\n";
		}
	}
}

void Image::postprocess()
{
	//Stopwatch
	//auto t_start = std::chrono::high_resolution_clock::now();

	correctInterleaved_();
	demultiplex_();				//Move the chuncks of data to the buffer array
	mTiff.mirrorOddFrames();	//The galvo (vectical axis of the image) performs bi-directional scanning. Divide the long vertical image in nFrames and vertically mirror the odd frames

	//Stop the stopwatch
	//double duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	//std::cout << "Post-processing elapsed time: " << duration << " ms" << "\n";

}

//Split the long vertical image into nFrames and calculate the average
void Image::averageFrames()
{
	mTiff.averageFrames();
}

//Divide the long image in nFrames, average the even and odd frames separately, and return the averages in separate pages
void Image::averageEvenOddFrames()
{
	mTiff.averageEvenOddFrames();
}

//Save each frame in mTiff in a single Tiff page
void Image::saveTiffSinglePage(std::string filename, const OverrideFileSelector overrideFlag, const ScanDirection stackScanDir) const
{
	mTiff.saveToFile(filename, SINGLEPAGE, overrideFlag, stackScanDir);
}

//Save each frame in mTiff in a different Tiff page
void Image::saveTiffMultiPage(std::string filename, const OverrideFileSelector overrideFlag, const ScanDirection stackScanDir) const
{
	mTiff.saveToFile(filename, MULTIPAGE, overrideFlag, stackScanDir);
}

//Access the Tiff data in the Image object
U8* const Image::pointerToTiff() const
{
	return mTiff.pointerToTiff();
}
#pragma endregion "Image"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const FPGAns::RTcontrol &RTcontrol): mRTcontrol(RTcontrol)
{	
	//Calculate the spatial fill factor
	const double temporalFillFactor{ mRTcontrol.mWidthPerFrame_pix * mRTcontrol.mDwell / halfPeriodLineclock };
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::voltageToI16(mControlVoltage)));
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::voltageToI16(mControlVoltage)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn(const double FFOV)
{
	setFFOV(FFOV);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV / um << " um\n";
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOnUsingVoltage(const double controlVoltage)
{
	setVoltage_(controlVoltage);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << controlVoltage / V << " V\n";
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(static_cast<DWORD>(mDelay/ms));
	setVoltage_(0);
	std::cout << "RS successfully turned off" << "\n";
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControlVoltage() const
{
	I16 control_I16;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadI16((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

	return FPGAns::I16toVoltage(control_I16);
}

//Check if the RS is set to run. It does not actually check if the RS is running, for example, by looking at the RSsync signal
void ResonantScanner::isRunning() const
{
	//Retrieve the state of the RS from the FPGA (see the LabView implementation)
	NiFpga_Bool isRunning{ false };

	char input_char;
	while (true)
	{
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadBool((mRTcontrol.mFpga).getHandle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));
		if (!isRunning)
		{
			std::cout << "RS seems OFF. Press ESC to exit or any other key to try again\n";
			input_char = _getch();

			if (input_char == 27)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		}
		else
			break; //break the whileloop

	}
}
#pragma endregion "Resonant scanner"


#pragma region "Galvo"
Galvo::Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel): mRTcontrol(RTcontrol), mGalvoRTchannel(galvoChannel)
{
	if ( mGalvoRTchannel != RTGALVO1 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
}

Galvo::Galvo(FPGAns::RTcontrol &RTcontrol, const RTchannel galvoChannel, const double posMax) : Galvo(RTcontrol, galvoChannel)
{
	generateFrameScan(posMax, -posMax);
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

//Generate a linear ramp to scan the galvo across a frame (a plane with fixed z)
void Galvo::generateFrameScan(const double xi, const double xf) const
{
	const double timeStep{ 8. * us };	//Time step of the linear ramp
	const double fineTuneHalfPeriodLineclock{ -0.58 * us };	//Adjust the RS half-period to fine tune the galvo's frame-scan
	const double frameDuration{ (halfPeriodLineclock + fineTuneHalfPeriodLineclock)  * mRTcontrol.mHeightPerFrame_pix };	//Time to scan a frame = (time for the RS to travel from side to side) x (# height of the frame in pixels)
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, frameDuration, mVoltagePerDistance * xi, mVoltagePerDistance * xf);
}

void Galvo::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mGalvoRTchannel, AO_tMIN, 0 * V);
}
#pragma endregion "Galvo"


#pragma region "PMT16X"
PMT16X::PMT16X()
{
	mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout/ms)));
}

PMT16X::~PMT16X()
{
	mSerial->close();
}

std::vector<uint8_t> PMT16X::sendCommand_(std::vector<uint8_t> command_array) const
{
	command_array.push_back(sumCheck_(command_array, command_array.size()));	//Append the sumcheck

	std::string TxBuffer{ command_array.begin(), command_array.end() }; //Convert the vector<char> to string
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
	uint8_t sum{ 0 };
	for (int ii = 0; ii < nElements; ii++)
		sum += charArray.at(ii);

	return sum;
}

void PMT16X::readAllGain() const
{
	std::vector<uint8_t> parameters{ sendCommand_({'I'})};

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


	std::vector<uint8_t> parameters{ sendCommand_({'g', (uint8_t)channel, (uint8_t)gain})};
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'g' && parameters.at(1) == (uint8_t)channel && parameters.at(2) == (uint8_t)gain && parameters.at(3) == sumCheck_(parameters, parameters.size()-2))
		std::cout << "PMT16X channel " << channel << " successfully set to " << gain << "\n";
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";
}

void PMT16X::setAllGainToZero() const
{
	std::vector<uint8_t> parameters{ sendCommand_({ 'R' }) }; //The manual says that this sets all the gains to 255, but it really does it to 0
															//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. The second char returned is the sumcheck
	if (parameters.at(0) == 'R' && parameters.at(1) == 'R')
		std::cout << "All PMT16X gains successfully set to 0\n";
}

void PMT16X::setAllGain(const int gain) const
{
	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain must be in the range 0-255");

	std::vector<uint8_t> parameters{ sendCommand_({ 'S', (uint8_t)gain }) };
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
	std::vector<uint8_t> parameters{ sendCommand_({ gains }) };
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
	std::vector<uint8_t> parameters{ sendCommand_({ 'T' }) };
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'T' || parameters.at(4) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	const int TEMPH{ static_cast<int>(parameters.at(1)) };
	const int TEMPL{ static_cast<int>(parameters.at(2)) };
	const double temp_C{ TEMPH + 0.01 * TEMPL }; //According to the manual

	const int alertTemp_C{ static_cast<int>(parameters.at(3)) };

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
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
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
	const std::string TxBuffer{ "pos?\r" };	//Command to the filterwheel
	std::string RxBuffer;					//Reply from the filterwheel

	try
	{
		mSerial->write(TxBuffer);
		mSerial->read(RxBuffer, mRxBufSize);
		//std::cout << "Full RxBuffer: " << RxBuffer << "\n"; //For debugging

		//Delete echoed command
		std::string::size_type ii{ RxBuffer.find(TxBuffer) };
		if (ii != std::string::npos)
			RxBuffer.erase(ii, TxBuffer.length());

		//Delete CR and >
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
		RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
		//RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

		//std::cout << "RxBuffer: " << RxBuffer << "\n";	//For debugging
		mPosition = std::stoi(RxBuffer);					//convert string to int
		mColor = positionToColor_(mPosition);
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
			const int minPos{ (std::min)(position, mPosition) };
			const int maxPos{ (std::max)(position, mPosition) };
			const int diffPos{ maxPos - minPos };
			const int minSteps{ (std::min)(diffPos, mNpos - diffPos) };

			//std::cout << "Tuning the " << mDeviceName << " to " + colorToString_(color) << "...\n";
			Sleep(static_cast<DWORD>(1. * minSteps / mTuningSpeed / ms));	//Wait until the filterwheel stops turning the turret

			mSerial->read(RxBuffer, mRxBufSize);		//Read RxBuffer to flush it. Serial::flush() doesn't work
														//std::cout << "setColor full RxBuffer: " << RxBuffer << "\n"; //For debugging

			downloadColor_();	//Check if the filterwheel was set successfully 

			if (position == mPosition)
			{
				//Thread-safe message
				std::stringstream msg;
				msg << mFilterwheelName << " successfully set to " + colorToString_(mColor) << " (position = " << mPosition << ")\n";
				std::cout << msg.str();
			}
			else
			{
				//Thread-safe message
				std::stringstream msg;
				msg << "WARNING: " << mFilterwheelName << " might not be in the correct position " << position << "\n";
				std::cout << msg.str();
			}
		}
		catch (const serial::IOException)
		{
			throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mFilterwheelName);
		}
	}
}

int Filterwheel::colorToPosition_(const Filtercolor color) const
{
	for (std::vector<int>::size_type iter = 0; iter != mFWconfig.size(); iter++)
	{
		if (color == mFWconfig.at(iter))
			return iter + 1;			//The index for mFWconfig starts from 0. The index for the filterwheel position start from 1
	}
	
	throw std::runtime_error((std::string)__FUNCTION__ + ": Failure converting color to position");
}

Filtercolor Filterwheel::positionToColor_(const int position) const
{
	if (position < 1 || position > mNpos)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": the filterwheel position must be between 1 and " + std::to_string(mNpos));

	return mFWconfig.at(position - 1);
}

//Convert from enum Filtercolor to string
std::string Filterwheel::colorToString_(const Filtercolor color) const
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

//Set the filter color using the laser wavelength
void Filterwheel::setWavelength(const int wavelength_nm)
{
	Filtercolor color;
	//Wavelength intervals chosen based on the 2p-excitation spectrum of the fluorescent labels (DAPI, GFP, and tdTomato)
	if (wavelength_nm > 940 && wavelength_nm <= 1080)
		color = RED;
	else if (wavelength_nm > 790)
		color = GREEN;
	else if (wavelength_nm >= 680)
		color = BLUE;
	else if (wavelength_nm == 0)
		color = NONE;
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The filterwheel wavelength must be in the range 680 - 1080 nm");

	setPosition_(colorToPosition_(color));
}
#pragma endregion "Filterwheel"

#pragma region "Laser"
Laser::Laser(const LaserSelector whichLaser): mWhichLaser(whichLaser)
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
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(mPort), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
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
			const std::string TxBuffer{ "?VW" };	//Command to the laser
			std::string RxBuffer;					//Reply from the laser
			mSerial->write(TxBuffer + "\r");
			mSerial->read(RxBuffer, mRxBufSize);

			//Delete echoed command. Echoing could be disabled on the laser side but deleting it is safer and more general
			std::string keyword{ "?VW " };
			std::string::size_type i{ RxBuffer.find(keyword) };
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

		if (wavelength_nm != mWavelength_nm)	//Set the new wavelength only if it is different from the current value
		{
			const std::string TxBuffer{ "VW=" + std::to_string(wavelength_nm) };	//Command to the laser
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
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": FIDELITY only supports the wavelenfth 1040 nm\n");
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

//Open or close the internal laser shutter
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
		std::string::size_type i{ RxBuffer.find(keyword) };
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
//To control the Uniblitz shutters
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, false));
}

void Shutter::setShutter(const bool state) const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, state));
}

void Shutter::pulse(const double pulsewidth) const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, true));

	Sleep(static_cast<DWORD>(pulsewidth/ms));

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, false));
}
#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Curently, output of the pockels cell is hardcoded on the FPGA side.  The pockels' output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LaserSelector laserSelector) :
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

double PockelsCell::laserpowerToVolt_(const double power) const
{
	double amplitude, angularFreq, phase;		//Calibration parameters

	//VISION
	switch (mPockelsRTchannel)
	{
	case RTVISION:
		if (mWavelength_nm == 750) {
			amplitude = 301.1 * mW;
			angularFreq = 0.624 / V;
			phase = 0.019 * V;
		}
		else if (mWavelength_nm == 920) {
			amplitude = 200.1 * mW;
			angularFreq = 0.507 / V;
			phase = -0.088 * V;
		}
		else if (mWavelength_nm == 1040) {
			amplitude = 75.19 * mW;
			angularFreq = 0.447 / V;
			phase = 0.038 * V;
		}
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		break;

		//FIDELITY
	case RTFIDELITY:
		amplitude = 210 * mW;
		angularFreq = 0.276 / V;
		phase = -0.049 * V;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	double arg{ sqrt(power / amplitude) };
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Arg of asin is greater than 1");

	return asin(arg) / angularFreq + phase;
}


void PockelsCell::pushVoltageSinglet(const double timeStep, const double AO, const OverrideFileSelector overrideFlag) const
{
	if (AO < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, AO, overrideFlag);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P, const OverrideFileSelector overrideFlag) const
{
	if (P < 0 || P > maxPower)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's laser power must be in the range 0-" + std::to_string(maxPower/mW));

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, laserpowerToVolt_(P), overrideFlag);
}

void PockelsCell::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, AO_tMIN, 0 * V);
}

//Linearly scale the laser power from Si to Sf across all the frames. Eg., Si = 1.0 and Sf = 2.0
void PockelsCell::scalingFactorLinearRamp_(const double Si, const double Sf) const
{
	if (Si < 0 || Sf < 0 || Si > 4 || Sf > 4)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested scaling factor must be in the range 0-4");

	if (mRTcontrol.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	mRTcontrol.clearQueue(mScalingRTchannel);	//Delete the default scaling factors = 1.0 created in the PockelsCell constructor

	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchannel, Si + (Sf - Si) / (mRTcontrol.mNframes - 1) * ii);
}

//Increase the pockels voltage linearly from the first to the last frame
void PockelsCell::voltageLinearRamp(const double Vi, const double Vf) const
{
	pushVoltageSinglet(timeStep, Vi, OVERRIDE);	//Set the laser power for the first frame
	scalingFactorLinearRamp_(1.0, Vf / Vi);		//Increase the laser power linearly from the first to the last frame
}

//Increase the laser power linearly from the first to the last frame
void PockelsCell::powerLinearRamp(const double Pi, const double Pf) const
{
	pushPowerSinglet(timeStep, Pi, OVERRIDE);	//Set the laser power for the first frame
	scalingFactorLinearRamp_(1.0, Pf / Pi);		//Increase the laser power linearly from the first to the last frame
}

void PockelsCell::setShutter(const bool state) const
{
	mShutter.setShutter(state);
}

/*
//Ramp up or down the pockels cell within a frame. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void PockelsCell::voltageLinearRampInFrame(const double timeStep, const double rampDuration, const double Vi, const double Vf) const
{
	if (Vi < 0 || Vf < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, Vi, Vf);
}

//Ramp up or down the pockels cell within a frame. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void  PockelsCell::powerLinearRampInFrame(const double timeStep, const double rampDuration, const double Pi, const double Pf) const
{
	if (Pi < 0 || Pf < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushLinearRamp(mPockelsRTchannel, timeStep, rampDuration, laserpowerToVolt_(Pi), laserpowerToVolt_(Pf));
}
*/
#pragma endregion "Pockels cells"


//Integrate the lasers, pockels cells, and filterwheels in a single class
#pragma region "VirtualLaser"
VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double initialPower, const double powerIncrease, const LaserSelector laserSelect) :
	mRTcontrol(RTcontrol), mLaserSelect(laserSelect), mVision(VISION), mFidelity(FIDELITY), mFWexcitation(FWEXC), mFWdetection(FWDET)
{
	//Select the laser to be used: VISION or FIDELITY
	mCurrentLaser = autoselectLaser_(wavelength_nm);
	std::cout << "Using " << laserNameToString_(mCurrentLaser) << " at " << wavelength_nm << " nm\n";

	//Update the laser wavelength if VISION was chosen
	if (mCurrentLaser == VISION)
		mVision.setWavelength(wavelength_nm);

	mPockelsPtr = std::unique_ptr<PockelsCell>(new PockelsCell(mRTcontrol, wavelength_nm, mCurrentLaser));	//Initialize the pockels cell
	setPower(initialPower, powerIncrease);																	//set the laser power

	isLaserInternalShutterOpen_();		//Check if the laser internal shutter is open
	turnFilterwheels_(wavelength_nm);	//Turn the filterwheels
}

VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double power, const LaserSelector laserSelect) : VirtualLaser(RTcontrol, wavelength_nm, power, 0, laserSelect) {}

VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LaserSelector laserSelect) : VirtualLaser(RTcontrol, wavelength_nm, 0, 0, laserSelect) {}

std::string VirtualLaser::laserNameToString_(const LaserSelector whichLaser) const
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

void VirtualLaser::isLaserInternalShutterOpen_() const
{
	while (true)
	{
		bool isShutterOpen;

		//Check which laser is being used
		switch (mCurrentLaser)
		{
		case VISION:
			isShutterOpen = mVision.isShutterOpen();
			break;
		case FIDELITY:
			isShutterOpen = mFidelity.isShutterOpen();
			break;
		}//switch

		//Check if the corresponding internal shutter is open
		if (!isShutterOpen)
		{
			std::cout << "The internal shutter of " + laserNameToString_(mCurrentLaser) + " seems to be closed. Press ESC to exit or any other key to try again\n";

			char input_char = _getch();
			if (input_char == 27)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		}
		else
			break; //break the whileloop
	}//whileloop
}

//Return VISION, FIDELITY, or let the code to decide
LaserSelector VirtualLaser::autoselectLaser_(const int wavelength_nm)
{
	//Update the wavelength to be used
	mWavelength_nm = wavelength_nm;

	//Use VISION for everything below 1040 nm. Use FIDELITY for 1040 nm	
	if (mLaserSelect == AUTO)
	{
		if (wavelength_nm < 1040)
			return VISION;
		else if (wavelength_nm == 1040)
			return FIDELITY;
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": wavelength > 1040 nm is not implemented in the VirtualLaser class");
	}
	else //If mLaserSelect != AUTO, the mLaserSelect is VISION or FIDELITY
		return mLaserSelect;
}

void VirtualLaser::turnFilterwheels_(const int wavelength_nm)
{
	//TO ALLOW MULTIPLEXING. For a single beam, Set the wavelength of the excitation filterwheel to 0, meaning that no beamsplitter is used
	int ExcWavelength_nm{ wavelength_nm };
	if (!mMultiplexing)
		ExcWavelength_nm = 0;

	//Turn both filterwheels concurrently
	std::thread th1{ &Filterwheel::setWavelength, &mFWexcitation, ExcWavelength_nm };
	std::thread th2{ &Filterwheel::setWavelength, &mFWdetection, wavelength_nm };
	th1.join();
	th2.join();
}

//Automatically select the laser for the requested wavelength. The pockels destructor closes the uniblitz shutter automatically to:
//1. switch from VISION to FIDELITY or vice versa
//2. avoid excessive photobleaching while tuning VISION
void VirtualLaser::setWavelength(const int wavelength_nm)
{
	//Ignore if the requested wavelength is current
	if (wavelength_nm != mWavelength_nm)
	{
		mCurrentLaser = autoselectLaser_(wavelength_nm);								//Update the selected laser
		mPockelsPtr.reset(new PockelsCell(mRTcontrol, wavelength_nm, mCurrentLaser));	//Update the pockels handler. The pockels destructor closes the uniblitz shutter automatically
		std::cout << "Using " << laserNameToString_(mCurrentLaser) << " at " << wavelength_nm << " nm\n";

		//If VISION is chosen, update the laser wavelength
		if (mCurrentLaser == VISION)
			mVision.setWavelength(wavelength_nm);

		isLaserInternalShutterOpen_();		//Check if the laser internal shutter is open
		turnFilterwheels_(wavelength_nm);	//Turn the filterwheels
	}
}

void VirtualLaser::setPower(const double initialPower, const double powerIncrease) const
{
	//Set the initial laser power
	mPockelsPtr->pushPowerSinglet(mPockelTimeStep, initialPower, OVERRIDE);

	//Set the power increase
	if (powerIncrease != 0)
		mPockelsPtr->powerLinearRamp(initialPower, initialPower + powerIncrease);
}

void VirtualLaser::openShutter() const
{
	mPockelsPtr->setShutter(true);
}

void VirtualLaser::closeShutter() const
{
	mPockelsPtr->setShutter(false);
}

#pragma endregion "VirtualLaser"

#pragma region "Stages"
Stage::Stage(const double velX, const double velY, const double velZ)
{
	const std::string stageIDx{ "116049107" };	//X-stage (V-551.4B)
	const std::string stageIDy{ "116049105" };	//Y-stage (V-551.2B)
	const std::string stageIDz{ "0165500631" };	//Z-stage (ES-100)

	//Open the connections to the stage controllers and assign the IDs
	std::cout << "Establishing connection with the stages\n";
	mID_XYZ.at(XX) = PI_ConnectUSB(stageIDx.c_str());
	mID_XYZ.at(YY) = PI_ConnectUSB(stageIDy.c_str());
	mID_XYZ.at(ZZ) = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID_XYZ.at(ZZ) = PI_ConnectUSB(stageIDz.c_str());

	if (mID_XYZ.at(XX) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");

	if (mID_XYZ.at(YY) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");

	if (mID_XYZ.at(ZZ) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	std::cout << "Connection with the stages successfully established\n";

	//Download the current position
	mPositionXYZ.at(XX) = downloadPositionSingle_(XX);
	mPositionXYZ.at(YY) = downloadPositionSingle_(YY);
	mPositionXYZ.at(ZZ) = downloadPositionSingle_(ZZ);

	//Download the current velocities
	mVelXYZ.at(XX) = downloadVelSingle_(XX);
	mVelXYZ.at(YY) = downloadVelSingle_(YY);
	mVelXYZ.at(ZZ) = downloadVelSingle_(ZZ);

	configDOtriggers_();				//Configure the stage velocities and DO triggers
	setVelXYZ({ velX, velY, velZ });	//Set the stage velocities
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mID_XYZ.at(XX));
	PI_CloseConnection(mID_XYZ.at(YY));
	PI_CloseConnection(mID_XYZ.at(ZZ));
	std::cout << "Connection to the stages successfully closed\n";
}


//DO1 and DO2 are used to trigger the stack acquisition. Currently only DO2 is used as trigger. See the implementation on LV
void Stage::configDOtriggers_() const
{
	/*
	//DO1 TRIGGER: DO1 is set to output a pulse (fixed width = 50 us) whenever the stage covers a certain distance (e.g. 0.3 um)
	const int DO1{ 1 };
	setDOtriggerEnabled(ZZ, DO1, true);	//Enable DO1 trigger
	const double triggerStep{ 0.3 * um };
	const StageDOtriggerMode triggerMode{ PositionDist };
	const double startThreshold{ 0. * mm };
	const double stopThreshold{ 0. * mm };
	setDOtriggerParamAll(ZZ, DO1, triggerStep, triggerMode, startThreshold, stopThreshold);
	*/

	//DO2 TRIGGER: DO2 is set to output HIGH when the stage z is in motion
	const int DO2{ 2 };
	setDOtriggerEnabled(ZZ, DO2, true);	//Enable DO2 trigger
	setDOtriggerParamSingle(ZZ, DO2, TriggerMode, InMotion);
}

std::string Stage::axisToString(const Axis axis) const
{
	switch (axis)
	{
	case XX:
		return "X";
	case YY:
		return "Y";
	case ZZ:
		return "Z";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid stage axis");
	}
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
double Stage::downloadPositionSingle_(const Axis axis)
{
	double position_mm;	//Position in mm
	if (!PI_qPOS(mID_XYZ.at(axis), mNstagesPerController, &position_mm))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query position for the stage " + axisToString(axis));

	return position_mm * mm;	//Multiply by mm to convert from explicit to implicit units
}

//Move the stage to the requested position
void Stage::moveSingle(const Axis axis, const double position)
{
	//Check if the requested position is within range
	if (position < mSoftPosLimXYZ.at(axis).at(0) || position > mSoftPosLimXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested position out of bounds for stage " + axisToString(axis));

	//Move the stage
	if (mPositionXYZ.at(axis) != position ) //Move only if the requested position is different from the current position
	{
		const double position_mm{ position / mm };								//Divide by mm to convert from implicit to explicit units
		if (!PI_MOV(mID_XYZ.at(axis), mNstagesPerController, &position_mm) )	//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage " + axisToString(axis) + " to the target position");

		mPositionXYZ.at(axis) = position;
	}
}

//Move the 3 stages to the requested position
void Stage::moveXY(const double2 positionXY)
{
	moveSingle(XX, positionXY.at(XX));
	moveSingle(YY, positionXY.at(YY));
}

//Move the 3 stages to the requested position
void Stage::moveXYZ(const double3 positionXYZ)
{
	moveSingle(XX, positionXYZ.at(XX));
	moveSingle(YY, positionXYZ.at(YY));
	moveSingle(ZZ, positionXYZ.at(ZZ));
}

bool Stage::isMoving(const Axis axis) const
{
	BOOL isMoving;

	if (!PI_IsMoving(mID_XYZ.at(axis), mNstagesPerController, &isMoving))	//~55 ms to execute this function
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage " + axisToString(axis));

	return isMoving;
}

void Stage::waitForMotionToStopSingle(const Axis axis) const
{
	std::cout << "Stage " + axisToString(axis) +  " moving to the new position: ";

	BOOL isMoving;
	do {
		if (!PI_IsMoving(mID_XYZ.at(axis), mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + axisToString(axis));

		std::cout << ".";
		Sleep(300);
	} while (isMoving);

	std::cout << "\n";
}

void Stage::waitForMotionToStopAll() const
{
	std::cout << "Stages moving to the new position: ";

	BOOL isMoving_x, isMoving_y, isMoving_z;
	do {
		if (!PI_IsMoving(mID_XYZ.at(XX), mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mID_XYZ.at(YY), mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mID_XYZ.at(ZZ), mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << ".";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}

void Stage::stopAll() const
{
	PI_StopAll(mID_XYZ.at(XX));
	PI_StopAll(mID_XYZ.at(YY));
	PI_StopAll(mID_XYZ.at(ZZ));

	std::cout << "Stages stopped\n";
}

//Request the velocity of the stage
double Stage::downloadVelSingle_(const Axis axis) const
{
	double vel_mmps;
	if (!PI_qVEL(mID_XYZ.at(axis), mNstagesPerController, &vel_mmps))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the velocity for the stage " + axisToString(axis));

	//std::cout << vel_mmps << " mm/s\n";
	return vel_mmps * mmps;					//Multiply by mmps to convert from explicit to implicit units
}

//Set the velocity of the stage
void Stage::setVelSingle(const Axis axis, const double vel)
{
	//Check if the requested vel is valid
	if (vel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The velocity must be greater than zero for the stage " + axisToString(axis));

	//Update the vel if different
	if (mVelXYZ.at(axis) != vel)
	{
		const double vel_mmps{ vel / mmps };		//Divide by mmps to convert implicit to explicit units
		if (!PI_VEL(mID_XYZ.at(axis), mNstagesPerController, &vel_mmps))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the velocity for the stage " + axisToString(axis));

		mVelXYZ.at(axis) = vel;
		//std::cout << "stage vel updated\n"; //For debugging
	}

}

//Set the velocity of the stage 
void Stage::setVelXYZ(const double3 velXYZ)
{
	setVelSingle(XX, velXYZ.at(XX));
	setVelSingle(YY, velXYZ.at(YY));
	setVelSingle(ZZ, velXYZ.at(ZZ));
}

void Stage::printVelXYZ() const
{
	std::cout << "Stage X vel = " << mVelXYZ.at(XX) / mmps << " mm/s\n";
	std::cout << "Stage Y vel = " << mVelXYZ.at(YY) / mmps << " mm/s\n";
	std::cout << "Stage Z vel = " << mVelXYZ.at(ZZ) / mmps << " mm/s\n";
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
double Stage::downloadDOtriggerParamSingle_(const Axis axis, const int DOchan, const StageDOparam param) const
{
	const int triggerParam{ static_cast<int>(param) };
	double value;
	if (!PI_qCTO(mID_XYZ.at(axis), &DOchan, &triggerParam, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger config for the stage " + axisToString(axis));

	//std::cout << value << "\n";
	return value;
}

void Stage::setDOtriggerParamSingle(const Axis axis, const int DOchan, const StageDOparam paramId, const double value) const
{
	const int triggerParam{ static_cast<int>(paramId) };
	if (!PI_CTO(mID_XYZ.at(axis), &DOchan, &triggerParam, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger config for the stage " + axisToString(axis));
}

void Stage::setDOtriggerParamAll(const Axis axis, const int DOchan, const double triggerStep, const StageDOtriggerMode triggerMode, const double startThreshold, const double stopThreshold) const
{
	if ( triggerStep <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The trigger step must be greater than zero");

	if (startThreshold < mTravelRangeXYZ.at(axis).at(0) || startThreshold > mTravelRangeXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": 'startThreshold is out of bound for the stage " + axisToString(axis));


	setDOtriggerParamSingle(axis, DOchan, TriggerStep, triggerStep / mm);					//Trigger step
	setDOtriggerParamSingle(axis, DOchan, AxisNumber, 1);									//Axis of the controller (always 1 because each controller has only 1 stage)
	setDOtriggerParamSingle(axis, DOchan, TriggerMode, static_cast<double>(triggerMode));	//Trigger mode
	setDOtriggerParamSingle(axis, DOchan, Polarity, 1);										//Polarity (0 for active low, 1 for active high)
	setDOtriggerParamSingle(axis, DOchan, StartThreshold, startThreshold / mm);				//Start threshold
	setDOtriggerParamSingle(axis, DOchan, StopThreshold, stopThreshold / mm);				//Stop threshold
}

//Request the enable/disable status of the stage DO
bool Stage::isDOtriggerEnabled(const Axis axis, const int DOchan) const
{
	BOOL triggerState;
	if (!PI_qTRO(mID_XYZ.at(axis), &DOchan, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger EN/DIS stage for the stage " + axisToString(axis));

	//std::cout << triggerState << "\n";
	return triggerState;
}

//Enable or disable the stage DO
void Stage::setDOtriggerEnabled(const Axis axis, const int DOchan, const BOOL triggerState) const
{
	if (!PI_TRO(mID_XYZ.at(axis), &DOchan, &triggerState, 1))
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

	const double triggerStep_mm{ downloadDOtriggerParamSingle_(axis, chan, TriggerStep) };
	const int triggerMode{ static_cast<int>(downloadDOtriggerParamSingle_(axis, chan, TriggerMode)) };
	const int polarity{ static_cast<int>(downloadDOtriggerParamSingle_(axis, chan, Polarity)) };
	const double startThreshold_mm{ downloadDOtriggerParamSingle_(axis, chan, StartThreshold) };
	const double stopThreshold_mm{ downloadDOtriggerParamSingle_(axis, chan, StopThreshold) };
	const double triggerPosition_mm{ downloadDOtriggerParamSingle_(axis, chan, TriggerPosition) };
	const bool triggerState{ isDOtriggerEnabled(axis, chan) };
	const double vel{ downloadVelSingle_(axis) };

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

#pragma region "Vibratome"
Vibratome::Vibratome(const FPGAns::FPGA &fpga, Stage &stage) : mFpga(fpga), mStage(stage) {}

//Start or stop running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::pushStartStopButton() const
{
	const int pulsewidth{ 100 * ms }; //in ms. It has to be longer than~ 12 ms, otherwise the vibratome is not triggered

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_VTstart, true));

	Sleep(static_cast<DWORD>(pulsewidth / ms));

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
}

void Vibratome::slice(const double planeToCutZ)
{
	mStage.setVelXYZ(mStageConveyingVelXYZ);															//Change the velocity to move the sample to the vibratome
	mStage.moveXYZ({ mStageInitialSlicePosXY.at(XX), mStageInitialSlicePosXY.at(YY), planeToCutZ });	//Position the sample in front of the vibratome's blade
	mStage.waitForMotionToStopAll();

	mStage.setVelSingle(YY, mSlicingVel);							//Change the y vel for slicing
	pushStartStopButton();											//Turn on the vibratome
	mStage.moveSingle(YY, mStageFinalSlicePosY);					//Slice the sample: move the stage y towards the blade
	mStage.waitForMotionToStopSingle(YY);							//Wait until the motion ends
	mStage.setVelSingle(YY, mStageConveyingVelXYZ.at(YY));			//Set back the y vel to move the sample back to the microscope

	//mStage.moveSingle(YY, mStage.mTravelRangeXYZ.at(YY).at(1));	//Move the stage y all the way to the end to push the cutoff slice forward, in case it gets stuck on the sample
	//mStage.waitForMotionToStopSingle(YY);							//Wait until the motion ends

	pushStartStopButton();											//Turn off the vibratome

}

/*//NOT USING THESE FUNCTIONS ANYMORE
//Move the head of the vibratome forward or backward for 'duration'. The timing varies in approx 1 ms
void Vibratome::moveHead_(const double duration, const MotionDir motionDir) const
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const double minDuration{ 10. * ms };
	const double delay{ 1. * ms };				//Used to roughly calibrate the pulse length

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

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), selectedChannel, true));

	if (duration >= minDuration)
		Sleep(static_cast<DWORD>((duration - delay)/ms));
	else
	{
		Sleep(static_cast<DWORD>((minDuration - delay)/ms));
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << 1. * minDuration / ms << "ms" << "\n";
	}
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), selectedChannel, false));
}

void Vibratome::cutAndRetractDistance(const double distance) const
{
	const double cuttingTime{ distance / mCuttingSpeed };
	const double retractingTime{ distance / mMovingSpeed };

	pushStartStopButton();
	std::cout << "The vibratome is cutting for " << cuttingTime / sec << " seconds" << "\n";
	Sleep(static_cast<DWORD>(cuttingTime / ms));
	Sleep(2000);
	std::cout << "The vibratome is retracting for " << retractingTime / sec << " seconds" << "\n";
	moveHead_(retractingTime, BACKWARD);
}


void Vibratome::retractDistance(const double distance) const
{
	const double retractingTime{ static_cast<int>(distance / mMovingSpeed) };
	std::cout << "The vibratome is retracting for " << retractingTime / sec << " seconds" << "\n";
	moveHead_(retractingTime, BACKWARD);
}
*/
#pragma endregion "Vibratome"


#pragma region "Sample"
Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, ROI roi, const double sampleLengthZ, const double sampleSurfaceZ, const double sliceOffset) :
	mName(sampleName), mImmersionMedium(immersionMedium), mObjectiveCollar(objectiveCollar), mROI(roi), mSurfaceZ(sampleSurfaceZ), mCutAboveBottomOfStack(sliceOffset)
{
	//Convert input ROI = {ymin, xmin, ymax, xmax} to the equivalent sample length in X and Y
	mLengthXYZ.at(XX) = mROI.at(XMAX) - mROI.at(XMIN);
	mLengthXYZ.at(YY) = mROI.at(YMAX) - mROI.at(YMIN);
	mLengthXYZ.at(ZZ) = sampleLengthZ;

	if (mLengthXYZ.at(XX) <= 0 || mLengthXYZ.at(YY) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": invalid ROI");

	if (mLengthXYZ.at(ZZ) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Z must be positive");

	if (mCutAboveBottomOfStack < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be positive");
}

Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar) :
	mName(sampleName), mImmersionMedium(immersionMedium), mObjectiveCollar(objectiveCollar), mCutAboveBottomOfStack(0) {}

void Sample::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SAMPLE ************************************************************\n";
	*fileHandle << "Name = " << mName << "\n";
	*fileHandle << "Immersion medium = " << mImmersionMedium << "\n";
	*fileHandle << "Correction collar = " << mObjectiveCollar << "\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "ROI [YMIN, XMIN, YMAX, XMAX] (mm) = [" << mROI.at(YMIN) / mm << "," << mROI.at(XMIN) / mm << "," << mROI.at(YMAX) / mm << "," << mROI.at(XMAX) / mm << "]\n";
	*fileHandle << "Length (mm) = (" << mLengthXYZ.at(XX) / mm << "," << mLengthXYZ.at(YY) / mm << "," << mLengthXYZ.at(ZZ) / mm << ")\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Blade position x,y (mm) = (" << mBladePositionXY.at(XX) / mm << "," << mBladePositionXY.at(YY) / mm << ")\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset (um) = " << mBladeFocalplaneOffsetZ / um << "\n";
	*fileHandle << "Cut above the bottom of the stack (um) = " << mCutAboveBottomOfStack / um << "\n";
	*fileHandle << "\n";
}
#pragma endregion "Sample"


#pragma region "Stack"
Stack::Stack(const double2 FFOV, const double stepSizeZ, const int nFrames, const double3 overlapXYZ_frac) :
	mFFOV(FFOV), mStepSizeZ(stepSizeZ), mDepth(stepSizeZ *  nFrames), mOverlapXYZ_frac(overlapXYZ_frac)
{
	if (FFOV.at(XX) <= 0 || FFOV.at(YY) <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");

	if (mStepSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The z-stage step size must be positive");

	if (mDepth <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth must be positive");

	if (mOverlapXYZ_frac.at(XX) < 0 || mOverlapXYZ_frac.at(YY) < 0 || mOverlapXYZ_frac.at(ZZ) < 0
		|| mOverlapXYZ_frac.at(XX) > 0.2 || mOverlapXYZ_frac.at(YY) > 0.2 || mOverlapXYZ_frac.at(ZZ) > 0.2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range 0-0.2%");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (um) = (" << mFFOV.at(XX) / um << "," << mFFOV.at(YY) / um << ")\n";
	*fileHandle << "Step size Z (um) = " << mStepSizeZ / um << "\n";
	*fileHandle << "Stack depth (um) = " << mDepth / um << "\n";
	*fileHandle << "Stack overlap (frac) = (" << mOverlapXYZ_frac.at(XX) << "," << mOverlapXYZ_frac.at(YY) << "," << mOverlapXYZ_frac.at(ZZ) << ")\n";
	*fileHandle << "Stack overlap (um) = (" << mOverlapXYZ_frac.at(XX) * mFFOV.at(XX) / um << "," << mOverlapXYZ_frac.at(YY) * mFFOV.at(YY) / um << "," << mOverlapXYZ_frac.at(ZZ) * mDepth << ")\n";
	*fileHandle << "\n";
}
#pragma endregion "Stack"

#pragma region "LaserList"
LaserList::LaserList(const std::vector <SingleLaser> laser) : mLaser(laser) {}

std::size_t LaserList::listSize() const
{
	return mLaser.size();
}

void LaserList::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "LASER ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mLaser.size(); iterWL++)
	{
		*fileHandle << "Wavelength (nm) = " << mLaser.at(iterWL).mWavelength_nm <<
			"\nLaser power (mW) = " << mLaser.at(iterWL).mScanPi / mW <<
			"\nPower increase (mW) = " << mLaser.at(iterWL).mStackPinc / mW << "\n";
	}
	*fileHandle << "\n";
}
#pragma endregion "LaserList"


/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/
