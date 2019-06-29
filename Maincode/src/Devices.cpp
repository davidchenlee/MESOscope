#include "Devices.h"

#pragma region "Image"

//When multiplexing, create a mTiff to contain 16 stripes of height 'mRTcontrol.mHeightPerFrame_pix' each
Image::Image(FPGAns::RTcontrol &RTcontrol) :
	mRTcontrol(RTcontrol), mTiff(mRTcontrol.mWidthPerFrame_pix, (static_cast<int>(multibeam) * 15 + 1) *  mRTcontrol.mHeightPerFrame_pix, mRTcontrol.mNframes)
{
	mBufArrayA = new U32[mRTcontrol.mNpixAllFrames];
	mBufArrayB = new U32[mRTcontrol.mNpixAllFrames];

	//Trigger the acquisition with the PC or the Z stage
	//This switch has to be here and not in the RTcontrol class because the z-trigger has to be turned off in the destructor to allow moving the z-stage
	if (static_cast<bool>(mRTcontrol.mMainTrigger))
		FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable, static_cast<bool>(mRTcontrol.mMainTrigger)));
}

Image::~Image()
{
	//Turn off the z-stage triggering the image acquisition to allow moving the z stage
	if (static_cast<bool>(mRTcontrol.mMainTrigger))
		FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable, false));

	//Before I implemented StopFIFOOUTpc_, the computer crashed every time the code was executed immediately after an exception.
	//I think this is because FIFOOUT used to remain open and clashed with the following call
	stopFIFOOUTpc_();

	delete[] mBufArrayA;
	delete[] mBufArrayB;

	//std::cout << "Image destructor called\n"; //For debugging
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
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, 0, timeout_ms, &nElemToReadA));
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, 0, timeout_ms, &nElemToReadB));
		//std::cout << "FIFOOUTpc cleanup A/B: " << nElemToReadA << "/" << nElemToReadB << "\n";
		//getchar();

		if (nElemToReadA == 0 && nElemToReadB == 0)
			break;

		if (nElemToReadA > 0)
		{
			nElemToReadA = min(bufSize, nElemToReadA);	//Min between bufSize and nElemToReadA
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, nElemToReadA, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalA += nElemToReadA;
		}
		if (nElemToReadB > 0)
		{
			nElemToReadB = min(bufSize, nElemToReadB);	//Min between bufSize and nElemToReadB
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, nElemToReadB, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
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
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), FIFOOUTpc, buffer, 0, timeout_ms, &nElemToRead));
		//std::cout << "Number of elements remaining in FIFOOUT: " << nElemToRead << "\n";	//For debugging

		//If data available in FIFOOUTpc, retrieve it
		if (nElemToRead > 0)
		{
			//If more data than expected
			if (static_cast<int>(nElemRead + nElemToRead) > mRTcontrol.mNpixAllFrames)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Received more FIFO elements than expected");

			//Retrieve the elements in FIFOOUTpc
			FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), FIFOOUTpc, buffer + nElemRead, nElemToRead, timeout_ms, &dummy));

			//Keep track of the total number of elements read
			nElemRead += nElemToRead;

			nullReadCounter = 0;	//Reset the iteration counter
		}
		else
			nullReadCounter++;		//keep track of the null reads
	}
}


//The RS scans bi-directionally. The pixel order has to be reversed for the odd or even lines. Currently I reverse the EVEN lines so that the resulting image matches the orientation of the sample
void Image::correctInterleaved_()
{
	//Within a line, the pixels go from lineIndex*widthPerFrame_pix to lineIndex*widthPerFrame_pix + widthPerFrame_pix - 1
	for (int lineIndex = 0; lineIndex < mRTcontrol.mHeightAllFrames_pix; lineIndex += 2)
	{
		std::reverse(mBufArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix, mBufArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix + mRTcontrol.mWidthPerFrame_pix);
		std::reverse(mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix, mBufArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix + mRTcontrol.mWidthPerFrame_pix);
	}
}

void Image::demultiplex_()
{	
	if (demuxAllPMTchan)
		demuxAllChannels_();
	else
		demuxSingleChannel_();
}

//Singlebeam. For speed, only process the counts from a single channel
void Image::demuxSingleChannel_()
{
	switch (mRTcontrol.mPMT16Xchan)
	{
	//Demultiplex mBufArrayA (channels 1-8). Each U32 element in mBufArrayA has the multiplexed structure | Ch08 (MSB) | Ch07 | Ch06 | Ch05 | Ch04 | Ch03 | Ch02 | Ch01 (LSB) |
	case PMT16XCHAN::CH01:
	case PMT16XCHAN::CH02:
	case PMT16XCHAN::CH03:
	case PMT16XCHAN::CH04:
	case PMT16XCHAN::CH05:
	case PMT16XCHAN::CH06:
	case PMT16XCHAN::CH07:
	case PMT16XCHAN::CH08:

		//Shift mBufArrayA to the right and extract the last 4 bits
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixAllFrames; pixIndex++)
			(mTiff.pointerToTiff())[pixIndex] = static_cast<U8>(mRTcontrol.mUpscaleFactorU8 * ((mBufArrayA[pixIndex] >> 4 * static_cast<unsigned char>(mRTcontrol.mPMT16Xchan)) & 0x0000000F));
		break;

	//Demultiplex mBufArrayB (channels 9-16). Each U32 element in mBufArrayB has the multiplexed structure | Ch16 (MSB) | Ch15 | Ch14 | Ch13 | Ch12 | Ch11 | Ch10 | Ch09 (LSB) |
	case PMT16XCHAN::CH09:
	case PMT16XCHAN::CH10:
	case PMT16XCHAN::CH11:
	case PMT16XCHAN::CH12:
	case PMT16XCHAN::CH13:
	case PMT16XCHAN::CH14:
	case PMT16XCHAN::CH15:
	case PMT16XCHAN::CH16:

		//Shift mBufArrayB to the right and extract the last 4 bits
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixAllFrames; pixIndex++)
			(mTiff.pointerToTiff())[pixIndex] = static_cast<U8>(mRTcontrol.mUpscaleFactorU8 * ((mBufArrayB[pixIndex] >> 4 * static_cast<unsigned char>(mRTcontrol.mPMT16Xchan)) & 0x0000000F));
		break;

	default:;//If CH00, don't do anything
	}
}


//Each U32 element in mBufArrayA and mBufArrayB has the multiplexed structure:
//mBufArrayA[i] =  | Ch08 (MSB) | Ch07 | Ch06 | Ch05 | Ch04 | Ch03 | Ch02 | Ch01 (LSB) |
//mBufArrayB[i] =  | Ch16 (MSB) | Ch15 | Ch14 | Ch13 | Ch12 | Ch11 | Ch10 | Ch09 (LSB) |


void Image::demuxAllChannels_()
{
	//Use 2 separate arrays to allow parallelization in the future
	TiffU8 CountA{ mRTcontrol.mWidthPerFrame_pix, 8 * mRTcontrol.mHeightPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in Ch01-Ch08
	TiffU8 CountB{ mRTcontrol.mWidthPerFrame_pix, 8 * mRTcontrol.mHeightPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in Ch09-Ch16

	/*Iterate over all the pixels and frames (all the frames are concatenated in a single-long image), demultiplex the counts, and store them in CountA and CountB
	CountA = |Ch01 f1|
			 |  .	 |
			 |Ch01 fN|
			 |  .	 |
			 |  .	 |
			 |  .	 |
			 |Ch08 f1|
			 |  .	 |
			 |Ch08 fN|

	CountB = |Ch09 f1|
			 |  .	 |
			 |Ch09 fN|
			 |  .	 |
			 |  .	 |
			 |  .	 |
			 |Ch16 f1|
			 |  .	 |
			 |Ch16 fN|
	*/
	for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixAllFrames; pixIndex++)
	{
		for (int channelIndex = 0; channelIndex < 8; channelIndex++)
		{
			//If upscaled overflows
			//if (upscaled > _UI8_MAX)
			//	upscaled = _UI8_MAX;

			//Extract the first 4 bits and upscale it
			(CountA.pointerToTiff())[channelIndex * mRTcontrol.mNpixAllFrames + pixIndex] = static_cast<U8>(mRTcontrol.mUpscaleFactorU8 * (mBufArrayA[pixIndex] & 0x0000000F));
			(CountB.pointerToTiff())[channelIndex * mRTcontrol.mNpixAllFrames + pixIndex] = static_cast<U8>(mRTcontrol.mUpscaleFactorU8 * (mBufArrayB[pixIndex] & 0x0000000F));

			//shift 4 places to the right
			mBufArrayA[pixIndex] = mBufArrayA[pixIndex] >> 4;
			mBufArrayB[pixIndex] = mBufArrayB[pixIndex] >> 4;
		}
	}

	//Merge the different PMT16X channels. The order depends on the scanning direction of the galvos (forward or backwards) and transfer the result to mTiff
	if (multibeam)
		mTiff.mergePMT16Xchannels(mRTcontrol.mHeightPerFrame_pix, CountA.pointerToTiff(), CountB.pointerToTiff()); //Here, mRTcontrol.mHeightPerFrame_pix is the height of a single PMT16X channel

	//For debugging
	if (saveTiffAllPMTchan)
	{
		//Save all the PMT16X channels in a different page of a Tiff
		TiffU8 stack{ mRTcontrol.mWidthPerFrame_pix, mRTcontrol.mHeightPerFrame_pix , 16 * mRTcontrol.mNframes };
		stack.pushImage(static_cast<int>(PMT16XCHAN::CH01), static_cast<int>(PMT16XCHAN::CH08), CountA.pointerToTiff());
		stack.pushImage(static_cast<int>(PMT16XCHAN::CH09), static_cast<int>(PMT16XCHAN::CH16), CountB.pointerToTiff());
		stack.saveToFile("AllChannels", MULTIPAGE::EN, OVERRIDE::DIS);
	}
}

//Establish a connection between FIFOOUTpc and FIFOOUTfpga and. Optional according to NI
void Image::startFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StartFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}

//Configure FIFOOUTpc. Optional according to NI
void Image::configureFIFOOUTpc_(const U32 depth) const
{
	U32 actualDepth;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << "\n";
}

//Stop the connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI
void Image::stopFIFOOUTpc_() const
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_StopFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
	//std::cout << "stopFIFO called\n";
}

void Image::acquire()
{
	initialize();
	mRTcontrol.triggerRT();			//Trigger the RT control. If triggered too early, FIFOOUTfpga will probably overflow

	if (static_cast<bool>(mRTcontrol.mFIFOOUTstate))
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

void Image::initialize(const ZSCAN stackScanDir)
{
	//Enable pushing data to FIFOOUTfpga. Disable for debugging
	if (static_cast<bool>(mRTcontrol.mFIFOOUTstate))
		FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTgateEnable, static_cast<bool>(mRTcontrol.mFIFOOUTstate)));

	//Initialize mScanDir
	mScanDir = stackScanDir;

	//Z STAGE. Fine tune the delay of the z-stage trigger for the acq sequence
	double ZstageTrigDelay{ 0 };

	if (mRTcontrol.mMainTrigger == MAINTRIG::ZSTAGE)
	{
		if (mRTcontrol.mHeightPerFrame_pix == 35)
		{
			switch (mScanDir)
			{
			case ZSCAN::TOPDOWN:
				ZstageTrigDelay = ZstageTrigDelayTopdown;
				break;
			case ZSCAN::BOTTOMUP:
				ZstageTrigDelay = ZstageTrigDelayBottomup;
				break;
			}
		}
		else if (mRTcontrol.mHeightPerFrame_pix >= 400) ; //Do nothing if mHeightPerFrame_pix is big enough
		else //ZstageTrigDelay is uncalibrated
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": ZstageTrigDelay has not been calibrated for heightPerFrame = " << mRTcontrol.mHeightPerFrame_pix << " pix\n";
			std::cerr << "Press any key to continue or ESC to exit\n";
			
			char input_char = _getch();
			if (input_char == 27)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		}

	}

	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlU32_ZstageTrigDelay_tick, static_cast<U32>(ZstageTrigDelay / us * tickPerUs)));

	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to the FPGA
	startFIFOOUTpc_();				//Establish connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI, but if not called, sometimes garbage is generated
	FIFOOUTpcGarbageCollector_();	//Cleans up any residual data from the previous run
}

void Image::downloadData()
{
	if (static_cast<bool>(mRTcontrol.mFIFOOUTstate))
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
	mTiff.mirrorOddFrames();	//The galvo (vectical axis of the image) performs bi-directional scanning. Divide the long vertical image in nFrames and mirror the odd frames vertically

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
void Image::saveTiffSinglePage(std::string filename, const OVERRIDE override) const
{
	mTiff.saveToFile(filename, MULTIPAGE::DIS, override, mScanDir);
}

//Save each frame in mTiff in a different Tiff page
void Image::saveTiffMultiPage(std::string filename, const OVERRIDE override) const
{
	mTiff.saveToFile(filename, MULTIPAGE::EN, override, mScanDir);
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::voltageToI16(mControlVoltage)));
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
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAns::voltageToI16(mControlVoltage)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn(const double FFOV)
{
	setFFOV(FFOV);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV / um << " um\n";
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOnUsingVoltage(const double controlVoltage)
{
	setVoltage_(controlVoltage);
	Sleep(static_cast<DWORD>(mDelay/ms));
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << controlVoltage / V << " V\n";
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAns::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(static_cast<DWORD>(mDelay/ms));
	setVoltage_(0);
	std::cout << "RS successfully turned off" << "\n";
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControlVoltage() const
{
	I16 control_I16;
	FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

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
		FPGAns::checkStatus(__FUNCTION__, NiFpga_ReadBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));
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
Galvo::Galvo(FPGAns::RTcontrol &RTcontrol, const RTCHAN channel, const int wavelength_nm): mRTcontrol(RTcontrol), mGalvoRTchannel(channel), mWavelength_nm(wavelength_nm)
{
	switch (channel)
	{
	case RTCHAN::SCANGALVO:
		mVoltagePerDistance = mScanCalib;
		break;
	case RTCHAN::RESCANGALVO:
		//Calibration factor to sync the rescanner with the scanner, and therefore, keep the fluorescence emission fixed at the detector
		//To find both parameters, image beads with a single laser beam at full FOV (i.e. 300x560 pixels) and look at the tiffs for all the channels
		//The beads should appear only in the channel selected
		//Adjust 'mVoltagePerDistance' until beads at different parts of the FOV are "synchronized" as the PMT16X channels are scrolled over
		//Adjust 'mRescanVoltageOffset' until the beads appear in the chosen PMT16X channel only
		switch (mWavelength_nm)
		{
		case 750:
			mVoltagePerDistance = 0.32 * mScanCalib;//larger number, top beads shows up first
			mRescanVoltageOffset = -0.00 * V;//A positive offset steers the beam towards channel 1

			break;
		case 920:
			mVoltagePerDistance = 0.32 * mScanCalib;
			mRescanVoltageOffset = 0.01 * V;
			break;
		case 1040:
			mVoltagePerDistance = 0.33 * mScanCalib;
			mRescanVoltageOffset = 0.03 * V;
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": galvo wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		}
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
	}
}

Galvo::Galvo(FPGAns::RTcontrol &RTcontrol, const RTCHAN channel, const double posMax, const int wavelength_nm) : Galvo(RTcontrol, channel, wavelength_nm)
{
	switch (channel)
	{
	case RTCHAN::SCANGALVO:
		positionLinearRamp(-posMax, posMax); //Raster scan from +x to -x wrt the x-stage axis
		break;
	case RTCHAN::RESCANGALVO:
		//Rescan in the direction opposite to the scan galvo to keep the fluorescent spot fixed at the detector
		positionLinearRamp(posMax, -posMax, mRescanVoltageOffset + beamletOrder.at(static_cast<int>(mRTcontrol.mPMT16Xchan)) * mInterBeamletDistance * mVoltagePerDistance);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
	}
}

void Galvo::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mGalvoRTchannel, AO_tMIN, 0 * V);
}

void Galvo::pushVoltageSinglet(const double timeStep, const double AO) const
{
	mRTcontrol.pushAnalogSinglet(mGalvoRTchannel, timeStep, AO);
}

void Galvo::voltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf) const
{
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, Vi, Vf);
}

void Galvo::positionLinearRamp(const double timeStep, const double rampLength, const double posInitial, const double posFinal) const
{
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, rampLength, mVoltagePerDistance * posInitial, mVoltagePerDistance * posFinal);
}

//Generate a linear ramp to scan the galvo across a frame (i.e., in a plane with fixed z)
void Galvo::positionLinearRamp(const double posInitial, const double posFinal, const double posOffset) const
{
	//Limit the number of steps for long ramps
	//Currently, the bottleneck is the buffe of the galvoss on the fpga that only support 5000 elements
	//For 2 us-steps, the max ramp duration is 10 ms. Therefore, 10 ms/ 62.5 us = 160 lines scanned
	double timeStep;
	if(mRTcontrol.mHeightPerFrame_pix <= 100)
		timeStep = 2. * us;
	else
		timeStep = 8. * us;

	//The position offset allows to compensate for the slight axis misalignment of the rescanner
	mRTcontrol.pushLinearRamp(mGalvoRTchannel, timeStep, halfPeriodLineclock * mRTcontrol.mHeightPerFrame_pix + mRampDurationFineTuning, 
		posOffset + mVoltagePerDistance * posInitial, posOffset + mVoltagePerDistance * posFinal);
}
#pragma endregion "Galvo"


#pragma region "PMT16X"
PMT16X::PMT16X()
{
	mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout/ms)));
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
Filterwheel::Filterwheel(const FILTERWHEEL whichFilterwheel): mWhichFilterwheel(whichFilterwheel)
{
	switch (whichFilterwheel)
	{
	case FILTERWHEEL::EXC:
		mPort = COM::FWEXC;
		mFilterwheelName = "Excitation filterwheel";
		mFWconfig = mExcConfig;								//Assign the filter positions
		break;
	case FILTERWHEEL::DET:
		mPort = COM::FWDET;
		mFilterwheelName = "Detection filterwheel";
		mFWconfig = mDetConfig;								//Assign the filter positions
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected filterwheel unavailable");
	}

	try
	{
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
		mPosition = downloadPosition_();		//Download the current filter position
		mColor = positionToColor_(mPosition);
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
int Filterwheel::downloadPosition_() const
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
		return std::stoi(RxBuffer);					//convert string to int
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the " + mFilterwheelName);
	}
}

int Filterwheel::colorToPosition_(const FILTERCOLOR color) const
{
	for (std::vector<int>::size_type iter = 0; iter < mFWconfig.size(); iter++)
	{
		if (color == mFWconfig.at(iter))
			return iter + 1;			//The index for mFWconfig starts from 0. The index for the filterwheel position start from 1
	}
	
	throw std::runtime_error((std::string)__FUNCTION__ + ": Failure converting color to position");
}

FILTERCOLOR Filterwheel::positionToColor_(const int position) const
{
	if (position < 1 || position > mNpos)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": the filterwheel position must be between 1 and " + std::to_string(mNpos));

	return mFWconfig.at(position - 1);
}

//Convert from enum FILTERCOLOR to string
std::string Filterwheel::colorToString_(const FILTERCOLOR color) const
{
	std::string colorStr;
	switch (color)
	{
	case FILTERCOLOR::BLUE:
		colorStr = "BLUE";
		break;
	case FILTERCOLOR::GREEN:
		colorStr = "GREEN";
		break;
	case FILTERCOLOR::RED:
		colorStr = "RED";
		break;
	case FILTERCOLOR::OPEN:
		colorStr = "OPEN";
		break;
	case FILTERCOLOR::CLOSED:
		colorStr = "CLOSED";
		break;
	default:
		colorStr = "UNKNOWN";
	}
	return colorStr;
}

void Filterwheel::setPosition(const FILTERCOLOR color)
{
	const int position = colorToPosition_(color);

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

			std::cout << "Tuning " << mFilterwheelName << " to " + colorToString_(color) << "...\n";
			Sleep(static_cast<DWORD>(1. * minSteps / mTuningSpeed / ms));	//Wait until the filterwheel stops turning the turret

			mSerial->read(RxBuffer, mRxBufSize);		//Read RxBuffer to flush it. Serial::flush() doesn't work
														//std::cout << "setColor full RxBuffer: " << RxBuffer << "\n"; //For debugging

			//Update the configuration of the filterwheel
			mPosition = downloadPosition_();  //Download the current filter position to check that the operation was successful
			mColor = color;

			if (position == mPosition)
			{
				//Thread-safe message
				std::stringstream msg;
				//msg << mFilterwheelName << " successfully set to " + colorToString_(mColor) << " (position = " << mPosition << ")\n";
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

//Set the filter color using the laser wavelength
void Filterwheel::setWavelength(const int wavelength_nm)
{
	FILTERCOLOR color;
	//Wavelength intervals chosen based on the 2p-excitation spectrum of the fluorescent labels (DAPI, GFP, and tdTomato)
	if (wavelength_nm > 940 && wavelength_nm <= 1080)
		color = FILTERCOLOR::RED;
	else if (wavelength_nm > 790)
		color = FILTERCOLOR::GREEN;
	else if (wavelength_nm >= 680)
		color = FILTERCOLOR::BLUE;
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The filterwheel wavelength must be in the range 680 - 1080 nm");

	setPosition(color);
}
#pragma endregion "Filterwheel"

#pragma region "Laser"
Laser::Laser(const LASER whichLaser): mWhichLaser(whichLaser)
{
	switch (mWhichLaser)
	{
	case LASER::VISION:
		laserName = "VISION";
		mPort = COM::VISION;
		mBaud = 19200;
		break;
	case LASER::FIDELITY:
		laserName = "FIDELITY";
		mPort = COM::FIDELITY;
		mBaud = 115200;
		break;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try
	{
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
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

int Laser::downloadWavelength_nm_() const
{
	switch (mWhichLaser)
	{
	case LASER::VISION:
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
	case LASER::FIDELITY:
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
	case LASER::VISION:
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

				mSerial->read(RxBuffer, mRxBufSize);		//Read RxBuffer to flush it. Serial::flush() doesn't work. The message reads "CHAMELEON>"

				mWavelength_nm = downloadWavelength_nm_();	//Check if the laser was set successfully 

				if (mWavelength_nm != wavelength_nm)
					std::cout << "WARNING: VISION might not be at the correct wavelength " << wavelength_nm << " nm\n";
			}
			catch (const serial::IOException)
			{
				throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
			}
		}
		break;
	case LASER::FIDELITY:
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
	case LASER::VISION:
		TxBuffer = "S=" + std::to_string(state);
		break;
	case LASER::FIDELITY:
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
	case LASER::VISION:
		TxBuffer = "?S";
		keyword = "?S ";
		break;
	case LASER::FIDELITY:
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
Shutter::Shutter(const FPGAns::FPGA &fpga, const LASER whichLaser) : mFpga(fpga)
{
	switch (whichLaser)
	{
	case LASER::VISION:
		mWhichShutter = NiFpga_FPGAvi_ControlBool_ShutterVision;
		break;
	case LASER::FIDELITY:
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
//Curently, the output of the pockels cell is gated on the FPGA side: the output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LASER laserSelector) :
	mRTcontrol(RTcontrol), mWavelength_nm(wavelength_nm), mShutter(mRTcontrol.mFpga, laserSelector)
{
	if (laserSelector != LASER::VISION && laserSelector != LASER::FIDELITY)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels channel unavailable");

	switch (laserSelector)
	{
	case LASER::VISION:
		mPockelsRTchannel = RTCHAN::VISION;
		mScalingRTchannel = RTCHAN::SCALINGVISION;
		break;
	case LASER::FIDELITY:
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength of FIDELITY can not be different from 1040 nm");
		mPockelsRTchannel = RTCHAN::FIDELITY;
		mScalingRTchannel = RTCHAN::SCALINGFIDELITY;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	//Initialize the power softlimit
#if multibeam
	//Multibeam		
	mMaxPower = 1600 * mW;
#else
	//Singlebeam			
	mMaxPower = 300 * mW;
#endif

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
	case RTCHAN::VISION:
		switch (mWavelength_nm)
		{
		case 750:
			amplitude = 1600.0 * mW;
			angularFreq = 0.624 / V;
			phase = 0.019 * V;
			break;
		case 920:
			amplitude = 1089.0 * mW;
			angularFreq = 0.507 / V;
			phase = -0.088 * V;
			break;
		case 1040:
			amplitude = 388.0 * mW;
			angularFreq = 0.447 / V;
			phase = 0.038 * V;
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		}			
		break;

		//FIDELITY
	case RTCHAN::FIDELITY:
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


void PockelsCell::pushVoltageSinglet(const double timeStep, const double AO, const OVERRIDE override) const
{
	if (AO < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, AO, override);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P, const OVERRIDE override) const
{
	if (P < 0 || P > mMaxPower)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's laser power must be in the range 0-" + std::to_string(static_cast<int>(mMaxPower/mW)) + " mW");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, timeStep, laserpowerToVolt_(P), override);
}

void PockelsCell::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mPockelsRTchannel, AO_tMIN, 0 * V);
}

//Increase the pockels voltage linearly from the first to the last frame
void PockelsCell::voltageLinearRamp(const double Vi, const double Vf) const
{
	const double Vratio = Vf / Vi;

	//Make sure that Fx2p14 will not overflow
	if (Vratio > 4)	
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested scaling factor must be in the range 0-4");

	if (mRTcontrol.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	pushVoltageSinglet(timeStep, Vi, OVERRIDE::EN);	//Set the laser power for the first frame

	//Delete the default scaling factors = 1.0 created in the PockelsCell constructor
	mRTcontrol.clearQueue(mScalingRTchannel);

	//Push the scaling factors
	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchannel, 1 + (Vratio - 1) / (mRTcontrol.mNframes - 1) * ii);
}

//Increase the laser power linearly from the first to the last frame
void PockelsCell::powerLinearRamp(const double Pi, const double Pf) const
{
	const double Vi = laserpowerToVolt_(Pi);
	const double Vf = laserpowerToVolt_(Pf);

	voltageLinearRamp(Vi, Vf);
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

#pragma region "CollectorLens"
CollectorLens::CollectorLens()
{	
	//Build list of connected device
	if (TLI_BuildDeviceList() == 0)
	{/*
		//Get device list size 
		short n = TLI_GetDeviceListSize();
		//std::cout << "Device list size: " << n << "\n";

		//Get BBD serial numbers
		char serialNos[100];
		//TLI_GetDeviceListByTypeExt(serialNos, 100, 80);

		//Output list of matching devices
		{
			char *searchContext = nullptr;
			char *p = strtok_s(serialNos, ",", &searchContext);
			
			while (p != nullptr)
			{
				TLI_DeviceInfo deviceInfo;
				//Get device info from device
				TLI_GetDeviceInfo(p, &deviceInfo);
				//Get strings from device info structure
				char desc[65];
				strncpy_s(desc, deviceInfo.description, 64);
				desc[64] = '\0';
				char serialNo[9];
				strncpy_s(serialNo, deviceInfo.serialNo, 8);
				serialNo[8] = '\0';
				//Output
				printf("Found Device %s=%s : %s\r\n", p, serialNo, desc);
				p = strtok_s(nullptr, ",", &searchContext);
			}
		}*/
	}

	//Open device
	if (SCC_Open(mSerialNumber))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Unable to establish connection with stepper " + mSerialNumber);

	Sleep(100);
	SCC_SetVelParams(mSerialNumber, mAcc_au, mVel_au);
}

CollectorLens::~CollectorLens()
{
	SCC_Close(mSerialNumber);	//Close device
}

void CollectorLens::move(const double position) const
{
	if (position < 0 || position > mPosLimit)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested position for the collector lens must be in the range 0-13 mm");

	//Start the device polling at 200ms intervals
	SCC_StartPolling(mSerialNumber, 200);

	//Move to position
	SCC_ClearMessageQueue(mSerialNumber);
	SCC_MoveToPosition(mSerialNumber, static_cast<int>(position * mCalib));
	std::cout << "Positioning the collector lens at " << position / mm << " mm...\n";

	//Wait for completion
	WORD messageType, messageId;
	DWORD messageData;
	SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
	while (messageType != 2 || messageId != 1)
	{
		SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
	}

	//Get current position
	//std::cout << "Collector lens current position: " << SCC_GetPosition(mSerialNumber) / mCalib /mm << " mm\n";

	//Stop polling
	SCC_StopPolling(mSerialNumber);
}

void CollectorLens::downloadConfig() const
{
	//Start the device polling at 200ms intervals
	SCC_StartPolling(mSerialNumber, 200);

	Sleep(100);	//The code does not work without this sleep
	std::cout << "Collector lens position: " << SCC_GetPosition(mSerialNumber) / mCalib / mm << " mm\n";

	int currentVelocity, currentAcceleration;
	SCC_GetVelParams(mSerialNumber, &currentAcceleration, &currentVelocity);
	std::cout << "Collector lens acceleration: " << currentAcceleration << "\tvelocity " << currentVelocity << "\n";

	//Stop polling
	SCC_StopPolling(mSerialNumber);
}

void CollectorLens::home() const
{
	//Start the device polling at 200ms intervals
	SCC_StartPolling(mSerialNumber, 200);

	Sleep(100);	//The code does not work without this sleep
	SCC_ClearMessageQueue(mSerialNumber);
	SCC_Home(mSerialNumber);
	std::cout << "Homing the collector lens\n";

	//Wait for completion
	WORD messageType, messageId;
	DWORD messageData;
	SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
	while (messageType != 2 || messageId != 0)
	{
		SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
	}

	//Stop polling
	SCC_StopPolling(mSerialNumber);
}
#pragma endregion "CollectorLens"

//Integrate the lasers, pockels cells, and filterwheels in a single class
#pragma region "VirtualLaser"
VirtualLaser::VirtualFilterWheel::VirtualFilterWheel() : mFWexcitation(FILTERWHEEL::EXC), mFWdetection(FILTERWHEEL::DET) {}

void VirtualLaser::VirtualFilterWheel::turnFilterwheels_(const int wavelength_nm)
{
#if multibeam
	//Multiplex
	//Turn both filterwheels concurrently
	std::thread th1{ &Filterwheel::setWavelength, &mFWexcitation, wavelength_nm };
	std::thread th2{ &Filterwheel::setWavelength, &mFWdetection, wavelength_nm };
	th1.join();
	th2.join();
#else
	//Single beam
	//Turn both filterwheels concurrently
	std::thread th1(&Filterwheel::setPosition, &mFWexcitation, FILTERCOLOR::OPEN);	//Set the excitation filterwheel open (no filter)
	std::thread th2(&Filterwheel::setWavelength, &mFWdetection, wavelength_nm);
	th1.join();
	th2.join();
#endif
}

VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double initialPower, const double finalPower, const LASER laserSelect) :
	mRTcontrol(RTcontrol), mLaserSelect(laserSelect), mVision(LASER::VISION), mFidelity(LASER::FIDELITY)
{
	//Tune the laser wavelength, set the excitation and emission filterwheels, and position the collector lens
	reconfigure(wavelength_nm);

	//Set the laser power
	setPower(initialPower, finalPower);
}

VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const double laserPower, const LASER laserSelect) : VirtualLaser(RTcontrol, wavelength_nm, laserPower, laserPower, laserSelect) {}

VirtualLaser::VirtualLaser(FPGAns::RTcontrol &RTcontrol, const int wavelength_nm, const LASER laserSelect) : VirtualLaser(RTcontrol, wavelength_nm, 0, 0, laserSelect) {}

std::string VirtualLaser::laserNameToString_(const LASER whichLaser) const
{
	switch (whichLaser)
	{
	case LASER::VISION:
		return "VISION";
	case LASER::FIDELITY:
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
		case LASER::VISION:
			isShutterOpen = mVision.isShutterOpen();
			break;
		case LASER::FIDELITY:
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

//Automatically select a laser: VISION, FIDELITY, or let the code to decide based on the requested wavelength
LASER VirtualLaser::autoSelectLaser_() const
{
	//Use VISION for everything below 1040 nm. Use FIDELITY for 1040 nm	
	if (mLaserSelect == LASER::AUTO)
	{
		if (mWavelength_nm < 1040)
			return LASER::VISION;
		else if (mWavelength_nm == 1040)
			return LASER::FIDELITY;
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": wavelength > 1040 nm is not implemented in the VirtualLaser class");
	}
	else //If mLaserSelect != LASER::AUTO, the mLaserSelect is either LASER::VISION or LASER::FIDELITY
		return mLaserSelect;
}

//Tune the laser wavelength (for VISION only)
void VirtualLaser::tuneLaserWavelength_()
{
	//Select the laser to be used: VISION or FIDELITY
	mCurrentLaser = autoSelectLaser_();
	std::cout << "Using " << laserNameToString_(mCurrentLaser) << " at " << mWavelength_nm << " nm\n";

	//If VISION is selected, update its wavelength
	if (mCurrentLaser == LASER::VISION)
		mVision.setWavelength(mWavelength_nm);

	//Update the pockels handler to initialize or update the laser power
	//The pockels destructor is made to close the uniblitz shutter automatically to allow switching between VISION and FIDELITY and also tuning VISION without photobleaching the sample
	mPockelsPtr.reset(new PockelsCell(mRTcontrol, mWavelength_nm, mCurrentLaser));
}

void VirtualLaser::positionCollectorLens_() const
{
	switch (mWavelength_nm)
	{
	case 750:
		mCollectorLens.move(10.0 * mm);
		break;
	case 920:
		mCollectorLens.move(4.0 * mm);
		break;
	case 1040:
		mCollectorLens.move(1.0 * mm);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Collector lens position has not been calibrated for the wavelength " + std::to_string(mWavelength_nm));
	}
}

//Tune the laser wavelength, set the exc and emission filterwheels, and position the collector lens
void VirtualLaser::reconfigure(const int wavelength_nm)
{
	//Ignore if the requested wavelength is current
	if (wavelength_nm != mWavelength_nm)
	{
		//Update the wavelength to be used
		mWavelength_nm = wavelength_nm;

		//Tune the laser wavelength
		tuneLaserWavelength_();
		//std::thread th1(tuneLaserWavelength_);

		//Set the filterwheels
		mVirtualFilterWheel.turnFilterwheels_(wavelength_nm);

		//Set the collector lens position
		positionCollectorLens_();

		//Check if the laser internal shutter is open
		isLaserInternalShutterOpen_();
	}
}

void VirtualLaser::setPower(const double laserPower) const
{
	//Set the initial laser power
	mPockelsPtr->pushPowerSinglet(mPockelTimeStep, laserPower, OVERRIDE::EN);
}

void VirtualLaser::setPower(const double initialPower, const double finalPower) const
{
	//Set the initial laser power
	mPockelsPtr->pushPowerSinglet(mPockelTimeStep, initialPower, OVERRIDE::EN);

	//Set the power increase
	if (finalPower != initialPower)
		mPockelsPtr->powerLinearRamp(initialPower, finalPower);
}

//Increase the laser power linearly from the first to the last frame
void VirtualLaser::powerLinearRamp(const double Pi, const double Pf) const
{
	mPockelsPtr->powerLinearRamp(Pi, Pf);
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
	//std::cout << "Connection with the stages successfully closed\n";
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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range 0-0.2");
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

#pragma region "ChannelList"
ChannelList::ChannelList(const std::vector<SingleChannel> channelList) : mList(channelList) {}

std::size_t ChannelList::size() const
{
	return mList.size();
}

ChannelList::SingleChannel ChannelList::front() const
{
	return mList.front();
}

ChannelList::SingleChannel ChannelList::at(const int index) const
{
	return mList.at(index);
}


void ChannelList::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "LASER ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mList.size(); iterWL++)
	{
		*fileHandle << "Wavelength (nm) = " << mList.at(iterWL).mWavelength_nm <<
			"\nLaser power (mW) = " << mList.at(iterWL).mScanPi / mW <<
			"\nPower increase (mW/um) = " << mList.at(iterWL).mStackPinc / mWpum << "\n";
	}
	*fileHandle << "\n";
}

//Return the first instance of "channel" in mList
ChannelList::SingleChannel ChannelList::findChannel(const std::string channel) const
{
	for (std::vector<int>::size_type iter_laser = 0; iter_laser < mList.size(); iter_laser++)
	{
		if (!channel.compare(mList.at(iter_laser).mName)) //compare() returns 0 if the strings are identical
			return mList.at(iter_laser);			
	}
	//If the requested channel is not found
	throw std::runtime_error((std::string)__FUNCTION__ + ": Channel " + channel + " not found");
}
#pragma endregion "ChannelList"


/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/
