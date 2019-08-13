#include "Devices.h"

#pragma region "Image"

//When multiplexing, create a mTiff to store 16 strips of height 'mRTcontrol.mHeightPerFrame_pix' each
Image::Image(const RTcontrol &RTcontrol) :
	mRTcontrol{ RTcontrol }, mTiff{ mRTcontrol.mWidthPerFrame_pix, (static_cast<int>(multibeam) * (nChanPMT - 1) + 1) *  mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes }
{
	mMultiplexedArrayA = new U32[mRTcontrol.mNpixPerBeamletAllFrames];
	mMultiplexedArrayB = new U32[mRTcontrol.mNpixPerBeamletAllFrames];

	//Trigger the data acquisition via the stage. It has to be here and not in the RTcontrol class because the trigger has to be turned off by the destructor to allow positioning the stage after acquisition
	mRTcontrol.enableStageTrigAcq();
}

Image::~Image()
{
	//Turn off the acq trigger by the z stage to allow moving the z stage
	mRTcontrol.disableStageTrigAcq();

	//Before I implemented StopFIFOOUTpc_, the computer crashed every time the code was executed immediately after an exception.
	//I think this is because FIFOOUT used to remain open and clashed with the following call
	stopFIFOOUTpc_();

	delete[] mMultiplexedArrayA;
	delete[] mMultiplexedArrayB;

	//std::cout << "Image destructor called\n"; //For debugging
}

//Access the Tiff data in the Image object
U8* const Image::data() const
{
	return mTiff.data();
}

//Scan a single frame
void Image::acquire(const bool saveAllPMT)
{
	initializeAcq();
	mRTcontrol.triggerRT();		//Send a signal to the FPGA to trigger the RT control. If triggered too early, FIFOOUTfpga will probably overflow
	downloadData();
	constructImage(saveAllPMT);
}

//Preset the parameters for the acquisition sequence
void Image::initializeAcq(const ZSCAN stackScanDir)
{
	//Push data to from the FPGA FIFOOUTfpga. Disabled when debugging
	mRTcontrol.enableFIFOOUT();

	//Initialize mScanDir
	mScanDir = stackScanDir;

	//Z STAGE. Fine tune the delay of the z-stage trigger for the acq sequence
	double ZstageTrigDelay{ 0 };
	if (mRTcontrol.mMainTrigger == MAINTRIG::ZSTAGE)
	{
		if (mRTcontrol.mHeightPerBeamletPerFrame_pix == 35)
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
		else if (mRTcontrol.mHeightPerBeamletPerFrame_pix >= 400)
			; //Do nothing if mHeightPerFrame_pix is big enough
		else //ZstageTrigDelay is uncalibrated
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": ZstageTrigDelay has not been calibrated for the heightPerFrame = " << mRTcontrol.mHeightPerBeamletPerFrame_pix << " pix\n";
			std::cerr << "Press any key to continue or ESC to exit\n";

			if (_getch() == 27)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		}
	}

	//Set the delay for the z-stage triggering the acq sequence
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlU32_ZstageTrigDelay_tick, static_cast<U32>(ZstageTrigDelay / us * tickPerUs)));

	mRTcontrol.presetFPGAoutput();	//Preset the ouput of the FPGA
	mRTcontrol.uploadRT();			//Load the RT control in mVectorOfQueues to be sent to the FPGA
	startFIFOOUTpc_();				//Establish connection between FIFOOUTpc and FIFOOUTfpga to send the RT control to the FGPA. Optional according to NI, but if not called, sometimes garbage is generated
	FIFOOUTpcGarbageCollector_();	//Clean up any residual data from the previous run
}

//Retrieve the data from the FPGA
void Image::downloadData()
{
	if (mRTcontrol.mFIFOOUTstate == FIFOOUT::EN)
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

//Demultiplex the image
void Image::constructImage(const bool saveAllPMT)
{
	correctInterleaved_();		//The RS scans bi-directionally. The pixel order has to be reversed either for the odd or even lines.
	demultiplex_(saveAllPMT);	//Move the chuncks of data to the buffer array
	mTiff.mirrorOddFrames();	//The galvo (vectical axis of the image) performs bi-directional scanning frame after frame. Divide the concatenated image in a stack with nFrames and mirror the odd frames vertically
}

//Image post processing
void Image::correctImage(const double FFOVfast)
{
	//mTiff.correctRSdistortionGPU(FFOVfast);		//Correct the image distortion induced by the nonlinear scanning of the RS

	if (multibeam)
	{
		//mTiff.suppressCrosstalk(0.1);
		//mTiff.flattenField(2.0);
	}
}

//Divide the concatenated image in a stack of nFrames and calculate the average over all the frames
void Image::averageFrames()
{
	mTiff.averageFrames();
}

//Divide the concatenated image in a stack of nFrames, average the even and odd frames separately, and return the averages in separate pages
void Image::averageEvenOddFrames()
{
	mTiff.averageEvenOddFrames();
}

//Divide the concatenated images into bins of nFramesPerBin frames and return a stack with the average in each bin
void Image::binFrames(const int nFramesPerBin)
{
	mTiff.binFrames(nFramesPerBin);
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

//Output true if the average count is below threshold
bool Image::isDark(const int threshold) const
{
	return mTiff.isDark(threshold);
}

//Flush the residual data in FIFOOUTpc from the previous run, if any
void Image::FIFOOUTpcGarbageCollector_() const
{
	const U32 timeout_ms{ 100 };
	const U32 bufSize{ 10000 };

	U32 dummy;
	U32* garbage{ new U32[bufSize] };
	U32 nElemToReadA{ 0 }, nElemToReadB{ 0 };			//Elements to read from FIFOOUTpc A and B
	int nElemTotalA{ 0 }, nElemTotalB{ 0 }; 			//Total number of elements read from FIFOOUTpc A and B
	while (true)
	{
		//Check if there are elements in FIFOOUTpc
		FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, 0, timeout_ms, &nElemToReadA));
		FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, 0, timeout_ms, &nElemToReadB));
		//std::cout << "FIFOOUTpc cleanup A/B: " << nElemToReadA << "/" << nElemToReadB << "\n";
		//getchar();

		if (nElemToReadA == 0 && nElemToReadB == 0)
			break;

		if (nElemToReadA > 0)
		{
			nElemToReadA = (std::min)(bufSize, nElemToReadA);	//Min between bufSize and nElemToReadA
			FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, garbage, nElemToReadA, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
			nElemTotalA += nElemToReadA;
		}
		if (nElemToReadB > 0)
		{
			nElemToReadB = (std::min)(bufSize, nElemToReadB);	//Min between bufSize and nElemToReadB
			FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, garbage, nElemToReadB, timeout_ms, &dummy));	//Retrieve the elements in FIFOOUTpc
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
	auto t_start{ std::chrono::high_resolution_clock::now() };
	*/
	
	const int readFifoWaitingTime_ms{ 5 };				//Waiting time between each iteration
	int timeout_iter{ 200 };							//Timeout the whileloop if the data transfer fails
	int nullReadCounterA{ 0 }, nullReadCounterB{ 0 };	//Null reading counters

	//FIFOOUT
	int nElemTotalA{ 0 }; 					//Total number of elements read from FIFOOUTpc A
	int nElemTotalB{ 0 }; 					//Total number of elements read from FIFOOUTpc B
	
	while (nElemTotalA < mRTcontrol.mNpixPerBeamletAllFrames || nElemTotalB < mRTcontrol.mNpixPerBeamletAllFrames)
	{
		Sleep(readFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time for max transfer bandwidth

		readChunk_(nElemTotalA, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, mMultiplexedArrayA, nullReadCounterA);	//FIFOOUTpc A
		readChunk_(nElemTotalB, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, mMultiplexedArrayB, nullReadCounterB);	//FIFOOUTpc B

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
	if (nElemTotalA <mRTcontrol.mNpixPerBeamletAllFrames || nElemTotalB < mRTcontrol.mNpixPerBeamletAllFrames)
		throw ImageException((std::string)__FUNCTION__ + ": Received less FIFO elements than expected");
}

//Read a chunk of data in the FIFOpc
void Image::readChunk_(int &nElemRead, const NiFpga_FPGAvi_TargetToHostFifoU32 FIFOOUTpc, U32* buffer, int &nullReadCounter)
{
	U32 dummy;
	U32 nElemToRead{ 0 };				//Elements remaining in FIFOOUTpc
	const U32 timeout_ms{ 100 };		//FIFOOUTpc timeout

	if (nElemRead < mRTcontrol.mNpixPerBeamletAllFrames)		//Skip if all the data have already been transferred
	{
		//By requesting 0 elements from FIFOOUTpc, the function returns the number of elements available. If no data is available, nElemToRead = 0 is returned
		FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), FIFOOUTpc, buffer, 0, timeout_ms, &nElemToRead));
		//std::cout << "Number of elements remaining in FIFOOUT: " << nElemToRead << "\n";	//For debugging

		//If data available in FIFOOUTpc, retrieve it
		if (nElemToRead > 0)
		{
			//If more data than expected
			if (static_cast<int>(nElemRead + nElemToRead) > mRTcontrol.mNpixPerBeamletAllFrames)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Received more FIFO elements than expected");

			//Retrieve the elements in FIFOOUTpc
			FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mRTcontrol.mFpga.getHandle(), FIFOOUTpc, buffer + nElemRead, nElemToRead, timeout_ms, &dummy));

			//Keep track of the total number of elements read
			nElemRead += nElemToRead;

			nullReadCounter = 0;	//Reset the iteration counter
		}
		else
			nullReadCounter++;		//keep track of the null reads
	}
}

//The RS scans bi-directionally. The pixel order has to be reversed either for the odd or even lines. Currently I reverse the EVEN lines so that the resulting image matches the orientation of the sample
void Image::correctInterleaved_()
{
	//std::reverse(mMultiplexedArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix, mMultiplexedArrayA + (lineIndex + 1) * mRTcontrol.mWidthPerFrame_pix)
	//reverses all the pixels between and including the indices 'lineIndex * widthPerFrame_pix' and '(lineIndex + 1) * widthPerFrame_pix - 1'
	for (int lineIndex = 0; lineIndex < mRTcontrol.mHeightPerBeamletAllFrames_pix; lineIndex += 2)
	{
		std::reverse(mMultiplexedArrayA + lineIndex * mRTcontrol.mWidthPerFrame_pix, mMultiplexedArrayA + (lineIndex + 1) * mRTcontrol.mWidthPerFrame_pix);
		std::reverse(mMultiplexedArrayB + lineIndex * mRTcontrol.mWidthPerFrame_pix, mMultiplexedArrayB + (lineIndex + 1) * mRTcontrol.mWidthPerFrame_pix);
	}
}

//Demultiplex the image
void Image::demultiplex_(const bool saveAllPMT)
{	
	if (multibeam || saveAllPMT)
		demuxAllChannels_(saveAllPMT);
	else
		demuxSingleChannel_();
}

//Singlebeam. Only readn and process the data from a single channel for speed
void Image::demuxSingleChannel_()
{
	//Shift mMultiplexedArrayA and  mMultiplexedArrayB to the right a number of bits depending on the PMT channel to be read
	//For mMultiplexedArrayA, shift 0 bits for CH00, 4 bits for CH01, 8 bits for CH02, etc...
	//For mMultiplexedArrayAB, shift 0 bits for CH08, 4 bits for CH09, 8 bits for CH10, etc...
	const unsigned int nBitsToShift{ 4 * static_cast<unsigned int>(mRTcontrol.mPMT16Xchan) };

	//Demultiplex mMultiplexedArrayA (CH00-CH07). Each U32 element in mMultiplexedArrayA has the multiplexed structure | CH07 (MSB) | CH06 | CH05 | CH04 | CH03 | CH02 | CH01 | CH00 (LSB) |
	if (mRTcontrol.mPMT16Xchan >= RTcontrol::PMT16XCHAN::CH00 && mRTcontrol.mPMT16Xchan <= RTcontrol::PMT16XCHAN::CH07)
	{
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixPerBeamletAllFrames; pixIndex++)
		{
			const int upscaled{ mRTcontrol.mUpscaleFactor * ((mMultiplexedArrayA[pixIndex] >> nBitsToShift) & 0x0000000F) };	//Extract the count from the last 4 bits and upscale it to have a 8-bit pixel
			(mTiff.data())[pixIndex] = clipU8top(upscaled);																		//Clip if overflow
		}
	}
	//Demultiplex mMultiplexedArrayB (CH08-CH15). Each U32 element in mMultiplexedArrayB has the multiplexed structure | CH15 (MSB) | CH14 | CH13 | CH12 | CH11 | CH10 | CH09 | CH08 (LSB) |
	else if (mRTcontrol.mPMT16Xchan >= RTcontrol::PMT16XCHAN::CH08 && mRTcontrol.mPMT16Xchan <= RTcontrol::PMT16XCHAN::CH15)
	{
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixPerBeamletAllFrames; pixIndex++)
		{
			const int upscaled{ mRTcontrol.mUpscaleFactor * ((mMultiplexedArrayB[pixIndex] >> nBitsToShift) & 0x0000000F) };	//Extract the count from the last 4 bits and upscale it to have a 8-bit pixel
			(mTiff.data())[pixIndex] = clipU8top(upscaled);																		//Clip if overflow
		}
	}
	else
		;//If PMT16XCHAN::CENTERED, do anything
}

//Each U32 element in mMultiplexedArrayA and mMultiplexedArrayB has the multiplexed structure:
//mMultiplexedArrayA[i] =  | CH07 (MSB) | CH06 | CH05 | CH04 | CH03 | CH02 | CH01 | CH00 (LSB) |
//mMultiplexedArrayB[i] =  | CH15 (MSB) | CH14 | CH13 | CH12 | CH11 | CH10 | CH09 | CH08 (LSB) |
void Image::demuxAllChannels_(const bool saveAllPMT)
{
	//Use 2 separate arrays to allow parallelization in the future
	TiffU8 CountA{ mRTcontrol.mWidthPerFrame_pix, 8 * mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in CH00-CH07
	TiffU8 CountB{ mRTcontrol.mWidthPerFrame_pix, 8 * mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in CH08-CH15

	/*Iterate over all the pixels and frames (all the frames are concatenated in a single-long image), demultiplex the counts, and store them in CountA and CountB
	CountA = |CH00 f1|
			 |  .	 |
			 |CH00 fN|
			 |  .	 |
			 |  .	 |
			 |  .	 |
			 |CH07 f1|
			 |  .	 |
			 |CH07 fN|

	CountB = |CH08 f1|
			 |  .	 |
			 |CH08 fN|
			 |  .	 |
			 |  .	 |
			 |  .	 |
			 |CH15 f1|
			 |  .	 |
			 |CH15 fN|
	*/

	for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixPerBeamletAllFrames; pixIndex++)
		for (int chanIndex = 0; chanIndex < 8; chanIndex++)
		{
			//Buffer A (CH00-CH07)
			const int upscaledA{ mRTcontrol.mUpscaleFactor * (mMultiplexedArrayA[pixIndex] & 0x0000000F) };			//Extract the count from the first 4 bits and upscale it to have a 8-bit pixel
			(CountA.data())[chanIndex * mRTcontrol.mNpixPerBeamletAllFrames + pixIndex] = clipU8top(upscaledA);		//Clip if overflow
			mMultiplexedArrayA[pixIndex] = mMultiplexedArrayA[pixIndex] >> 4;										//Shift 4 places to the right for the next iteration

			//Buffer B (CH08-CH15)
			const int upscaledB{ mRTcontrol.mUpscaleFactor * (mMultiplexedArrayB[pixIndex] & 0x0000000F) };			//Extract the count from the first 4 bits and upscale it to have a 8-bit pixel
			(CountB.data())[chanIndex * mRTcontrol.mNpixPerBeamletAllFrames + pixIndex] = clipU8top(upscaledB);		//Clip if overflow
			mMultiplexedArrayB[pixIndex] = mMultiplexedArrayB[pixIndex] >> 4;										//Shift 4 places to the right for the next iteration
		}

	//Merge all the PMT16X channels into a single image. The strip ordering depends on the scanning direction of the galvos (forward or backwards)
	if (multibeam)
		mTiff.mergePMT16Xchan(mRTcontrol.mHeightPerBeamletPerFrame_pix, CountA.data(), CountB.data());				//mHeightPerBeamletPerFrame_pix is the height for a single PMT16X channel

	//For debugging
	if (saveAllPMT)
	{
		//Save all PMT16X channels in separate pages in a Tiff
		TiffU8 stack{ mRTcontrol.mWidthPerFrame_pix, mRTcontrol.mHeightPerBeamletPerFrame_pix , nChanPMT * mRTcontrol.mNframes };
		stack.pushImage(static_cast<int>(RTcontrol::PMT16XCHAN::CH00), static_cast<int>(RTcontrol::PMT16XCHAN::CH07), CountA.data());
		stack.pushImage(static_cast<int>(RTcontrol::PMT16XCHAN::CH08), static_cast<int>(RTcontrol::PMT16XCHAN::CH15), CountB.data());

		std::string PMT16Xchan_s{ std::to_string(static_cast<int>(mRTcontrol.mPMT16Xchan)) };
		stack.saveToFile("AllChannels PMT16Xchan=" + PMT16Xchan_s, MULTIPAGE::EN, OVERRIDE::DIS);
	}
}

//Establish a connection between FIFOOUTpc and FIFOOUTfpga and. Optional according to NI
void Image::startFIFOOUTpc_() const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_StartFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_StartFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}

//Configure FIFOOUTpc. Optional according to NI
void Image::configureFIFOOUTpc_(const U32 depth) const
{
	U32 actualDepth;
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << "\n";
}

//Stop the connection between FIFOOUTpc and FIFOOUTfpga. Optional according to NI
void Image::stopFIFOOUTpc_() const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_StopFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_StopFifo(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
	//std::cout << "stopFIFO called\n";
}
#pragma endregion "Image"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const RTcontrol &RTcontrol) : mRTcontrol{ RTcontrol }
{	
	//Calculate the spatial fill factor
	const double temporalFillFactor{ mRTcontrol.mWidthPerFrame_pix * pixelDwellTime / lineclockHalfPeriod };
	if (temporalFillFactor > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");
	else
		mFillFactor = sin(PI / 2 * temporalFillFactor);						//Note that the fill factor doesn't depend on the RS amplitude
																			//because the RS period is always the same and independent of the amplitude

	//std::cout << "Fill factor = " << mFillFactor << "\n";					//For debugging

	//Download the current control voltage from the FPGA and update the scan parameters
	mControlVoltage = downloadControlVoltage();								//Control voltage
	mFullScan = mControlVoltage / mVoltagePerDistance;						//Full scan FOV = distance between the turning points
	mFFOV = mFullScan * mFillFactor;										//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;						//Spatial sampling resolution (length/pixel)
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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested FFOV must be in the range [0-" + std::to_string(mVMAX / mVoltagePerDistance / um) + "] um");

	//Upload the control voltage
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAfunc::voltageToI16(mControlVoltage)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn(const double FFOV)
{
	setFFOV(FFOV);
	Sleep(static_cast<DWORD>(mDelay / ms));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV / um << " um\n";
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOnUsingVoltage(const double controlVoltage)
{
	setVoltage_(controlVoltage);
	Sleep(static_cast<DWORD>(mDelay / ms));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << controlVoltage / V << " V\n";
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(static_cast<DWORD>(mDelay / ms));
	setVoltage_(0);
	std::cout << "RS successfully turned off" << "\n";
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControlVoltage() const
{
	I16 control_I16;
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

	return FPGAfunc::intToVoltage(control_I16);
}

//Check if the RS is set to run. It does not actually check if the RS is running, for example, by looking at the RSsync signal
void ResonantScanner::isRunning() const
{
	//Retrieve the state of the RS from the FPGA (see the LabView implementation)
	NiFpga_Bool isRunning{ false };

	char input_char;
	while (true)
	{
		FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadBool(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));
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

//Set the control voltage that determines the scanning amplitude
void ResonantScanner::setVoltage_(const double controlVoltage)
{
	if (controlVoltage < 0 || controlVoltage > mVMAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested voltage must be in the range [0-" + std::to_string(mVMAX) + "] V" );

	//Update the scan parameters
	mControlVoltage = controlVoltage;									//Control voltage
	mFullScan = controlVoltage / mVoltagePerDistance;					//Full scan FOV
	mFFOV = mFullScan * mFillFactor;									//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;					//Spatial sampling resolution (length/pixel)

	//Upload the control voltage
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAfunc::voltageToI16(mControlVoltage)));
}
#pragma endregion "Resonant scanner"

#pragma region "PMT16X"
PMT16X::PMT16X()
{
	mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout/ms)));
}

PMT16X::~PMT16X()
{
	mSerial->close();
}

void PMT16X::readAllGain() const
{
	std::vector<uint8_t> parameters{ sendCommand_({'I'}) };

	//The gains are stored in parameters.at(1) to parameters.at(15)
	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two elements in 'parameter', which are the returned sumcheck and CR
	if (parameters.at(0) != 'I' || parameters.at(17) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	//Print out the gains
	std::cout << "PMT16X gains:\n";
	for (int ii = 0; ii < nChanPMT; ii++)
		std::cout << "Gain CH" << ii << " (0-255) = " << static_cast<int>(parameters.at(ii + 1)) << "\n";
}

void PMT16X::setSingleGain(const RTcontrol::PMT16XCHAN chan, const int gain) const
{
	//Check that the inputVector parameters are within range
	if (chan < RTcontrol::PMT16XCHAN::CH00 || chan > RTcontrol::PMT16XCHAN::CH15)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X channel number out of range [1-" + std::to_string(nChanPMT) + "]");

	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain out of range [0-255]");

	//The PMT16X indexes the channels starting from 1 to 16
	uint8_t chanPMT{ static_cast<uint8_t>(static_cast<int>(chan) + 1) };

	std::vector<uint8_t> parameters{ sendCommand_({'g', chanPMT, (uint8_t)gain}) };
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'g' && parameters.at(1) == chanPMT && parameters.at(2) == (uint8_t)gain && parameters.at(3) == sumCheck_(parameters, parameters.size() - 2))
		std::cout << "PMT16X channel " << static_cast<int>(chan) << " successfully set to " << gain << "\n";
	else
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";
}

void PMT16X::setAllGainToZero() const
{
	std::vector<uint8_t> parameters{ sendCommand_({ 'R' }) };	//The manual says that this sets all the gains to 255, but it really does it to 0
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. The second char returned is the sumcheck
	if (parameters.at(0) == 'R' && parameters.at(1) == 'R')
		std::cout << "All PMT16X gains successfully set to 0\n";
}

void PMT16X::setAllGain(const int gain) const
{
	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X gain must be in the range [0-255]");

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
	if (gains.size() != nChanPMT)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Gain array must have " + std::to_string(nChanPMT) + " elements");

	for (int ii = 0; ii < nChanPMT; ii++)
		if (gains.at(ii) < 0 || gains.at(ii) > 255)
			throw std::invalid_argument((std::string)__FUNCTION__ + ":  PMT16X gain #" + std::to_string(ii) + " out of range [0-255]");

	gains.insert(gains.begin(), 'G');	//Prepend the command
	std::vector<uint8_t> parameters{ sendCommand_({ gains }) };
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'G' || parameters.at(17) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	//Print out the gains
	std::cout << "PMT16X gains successfully set to:\n";
	for (int ii = 1; ii <= nChanPMT; ii++)
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
	std::cout << "PMT16X alert temperature = " << alertTemp_C << " \370C\n";
}

std::vector<uint8_t> PMT16X::sendCommand_(std::vector<uint8_t> command_array) const
{
	command_array.push_back(sumCheck_(command_array, command_array.size()));	//Append the sumcheck

	std::string TxBuffer{ command_array.begin(), command_array.end() };			//Convert the vector<char> to string
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
#pragma endregion "PMT16X"


#pragma region "Filterwheel"
Filterwheel::Filterwheel(const ID whichFilterwheel) : mWhichFilterwheel{ whichFilterwheel }
{
	switch (whichFilterwheel)
	{
	case ID::EXC:
		mPort = COM::FWEXC;
		mFilterwheelName = "Excitation filterwheel";
		mFWconfig = mExcConfig;								//Assign the filter positions
		break;
	case ID::DET:
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

//Set the color of the filterwheel
void Filterwheel::setColor(const COLOR color)
{
	const int position{ colorToPosition_(color) };

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

			//Thread-safe message
			std::stringstream msg;
			msg << "Setting the " << mFilterwheelName << " to " + colorToString_(color) << "...\n";
			std::cout << msg.str();

			Sleep(static_cast<DWORD>(1. * minSteps / mTurningSpeed / ms));	//Wait until the filterwheel stops turning the turret

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

//Set the filter color by specifying the laser wavelength
void Filterwheel::setWavelength(const int wavelength_nm)
{
	COLOR color;
	//Wavelength intervals chosen based on the 2p-excitation spectrum of the fluorescent labels (DAPI, GFP, and tdTomato)
	if (wavelength_nm > 940 && wavelength_nm <= 1080)
		color = COLOR::RED;
	else if (wavelength_nm > 790)
		color = COLOR::GREEN;
	else if (wavelength_nm >= 680)
		color = COLOR::BLUE;
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The filterwheel wavelength must be in the range [680-1080] nm");

	setColor(color);
}

//Download the current filter position
int Filterwheel::downloadPosition_() const
{
	try
	{
		const std::string TxBuffer{ "pos?\r" };	//Command to the filterwheel
		std::string RxBuffer;					//Reply from the filterwheel

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

//Convert the color of the filter to the wheel position
int Filterwheel::colorToPosition_(const COLOR color) const
{
	for (std::vector<int>::size_type iter = 0; iter < mFWconfig.size(); iter++)
	{
		if (color == mFWconfig.at(iter))
			return iter + 1;			//The index for mFWconfig starts from 0. The index for the filterwheel position start from 1
	}
	
	throw std::runtime_error((std::string)__FUNCTION__ + ": Failure converting color to position");
}

//Convert the wheel position to the color
Filterwheel::COLOR Filterwheel::positionToColor_(const int position) const
{
	if (position < 1 || position > mNpos)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The filterwheel position must be between 1 and " + std::to_string(mNpos));

	return mFWconfig.at(position - 1);
}

//Convert enum COLOR to string
std::string Filterwheel::colorToString_(const COLOR color) const
{
	std::string colorStr;
	switch (color)
	{
	case COLOR::BLUE:
		colorStr = "BLUE";
		break;
	case COLOR::GREEN:
		colorStr = "GREEN";
		break;
	case COLOR::RED:
		colorStr = "RED";
		break;
	case COLOR::OPEN:
		colorStr = "OPEN";
		break;
	case COLOR::CLOSED:
		colorStr = "CLOSED";
		break;
	default:
		colorStr = "UNKNOWN";
	}
	return colorStr;
}
#pragma endregion "Filterwheel"

#pragma region "Laser"
Laser::Laser(const ID whichLaser) : mWhichLaser{ whichLaser }
{
	switch (mWhichLaser)
	{
	case ID::VISION:
		laserName = "VISION";
		mPort = COM::VISION;
		mBaud = 19200;
		break;
	case ID::FIDELITY:
		laserName = "FIDELITY";
		mPort = COM::FIDELITY;
		mBaud = 115200;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}

	try //Establish serial communication with the chosen laser
	{
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure establishing serial communication with " + laserName);
	}

	//Store the wavelength as a class member
	mWavelength_nm = downloadWavelength_nm_();
}

Laser::~Laser()
{
	mSerial->close();
}

void Laser::printWavelength_nm() const
{
	std::cout << laserName + " wavelength is " << mWavelength_nm << " nm\n";
}

//Set the wavelength of the laser (Vision only)
void Laser::setWavelength(const int wavelength_nm)
{
	switch (mWhichLaser)
	{
	case ID::VISION:
		if (wavelength_nm < 680 || wavelength_nm > 1080)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": VISION wavelength must be in the range [680-1080] nm");

		if (wavelength_nm != mWavelength_nm)	//Change the wavelength only if the new value is different from the current one
		{
			const std::string TxBuffer{ "VW=" + std::to_string(wavelength_nm) };	//Command to the laser
			std::string RxBuffer;													//Reply from the laser

			try
			{
				mSerial->write(TxBuffer + "\r");
				//std::cout << "Sleep time in ms: " << static_cast<int>( std::abs( 1.*(mWavelength_nm - wavelength_nm) / mTuningSpeed / ms ) ) << "\n";	//For debugging

				//Thread-safe message
				std::stringstream msg;
				msg << "Tuning VISION to " << wavelength_nm << " nm...\n";
				std::cout << msg.str();

				//Wait till the laser stops tuning
				Sleep(static_cast<DWORD>(std::abs(1.*(mWavelength_nm - wavelength_nm) / mTuningSpeed / ms)));

				//Read RxBuffer to flush it. Serial::flush() doesn't work. The message reads "CHAMELEON>"
				mSerial->read(RxBuffer, mRxBufSize);	

				//Check if the laser was set successfully 
				mWavelength_nm = downloadWavelength_nm_();

				if (mWavelength_nm != wavelength_nm)
				{
					//Thread-safe message
					std::stringstream msg;
					msg << "WARNING: VISION might not be at the correct wavelength " << wavelength_nm << " nm\n";
					std::cout << msg.str();
				}
			}
			catch (const serial::IOException)
			{
				throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
			}
		}
		break;
	case ID::FIDELITY:
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": FIDELITY only supports the wavelength 1040 nm\n");
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
	case ID::VISION:
		TxBuffer = "S=" + std::to_string(state);
		break;
	case ID::FIDELITY:
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
	case ID::VISION:
		TxBuffer = "?S";
		keyword = "?S ";
		break;
	case ID::FIDELITY:
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

int Laser::currentWavelength_nm() const
{
	return mWavelength_nm;
}

//Download the current wavelength of the laser (Vision only)
int Laser::downloadWavelength_nm_() const
{
	switch (mWhichLaser)
	{
	case ID::VISION:
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

			//Delete '\r' and '\n'
			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
			RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());
			//std::cout << RxBuffer << "\n";	//For debugging

			return std::stoi(RxBuffer);	//Convert string to int

		}
		catch (const serial::IOException)
		{
			throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with VISION");
		}
	case ID::FIDELITY:
		return 1040;
	default:
		throw std::runtime_error((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}	
}
#pragma endregion "Laser"

#pragma region "Shutters"
//To control the Uniblitz shutters
Shutter::Shutter(const FPGA &fpga, const Laser::ID whichLaser) : mFpga{ fpga }
{
	switch (whichLaser)
	{
	case Laser::ID::VISION:
		mWhichShutter = NiFpga_FPGAvi_ControlBool_ShutterVision;
		break;
	case Laser::ID::FIDELITY:
		mWhichShutter = NiFpga_FPGAvi_ControlBool_ShutterFidelity;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected shutter unavailable");
	}
}

Shutter::~Shutter()
{
	//This is to prevent keeping the shutter open in case of an exception
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, false));
}

//Set the shutter open or close
void Shutter::setShutter(const bool state) const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, state));
}

//Open and close the shutter
void Shutter::pulse(const double pulsewidth) const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, true));

	Sleep(static_cast<DWORD>(pulsewidth/ms));

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), mWhichShutter, false));
}
#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Curently, the output of the pockels cell is gated on the FPGA side: the output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(RTcontrol &RTcontrol, const int wavelength_nm, const Laser::ID laserSelector) :
	mRTcontrol{ RTcontrol }, mWavelength_nm{ wavelength_nm }, mShutter{ mRTcontrol.mFpga, laserSelector }
{
	if (laserSelector != Laser::ID::VISION && laserSelector != Laser::ID::FIDELITY)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels channel unavailable");

	switch (laserSelector)
	{
	case Laser::ID::VISION:
		mPockelsRTchan = RTcontrol::RTCHAN::VISION;
		mScalingRTchan = RTcontrol::RTCHAN::SCALINGVISION;
		break;
	case Laser::ID::FIDELITY:
		if (wavelength_nm != 1040)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength of FIDELITY can not be different from 1040 nm");
		mPockelsRTchan = RTcontrol::RTCHAN::FIDELITY;
		mScalingRTchan = RTcontrol::RTCHAN::SCALINGFIDELITY;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	//Initialize the power softlimit
#if multibeam
	mMaxPower = 1000 * mW;//Multibeam		
#else	
	mMaxPower = 150 * mW;//Singlebeam	
#endif

	//Initialize all the scaling factors to 1.0. In LV, I could not sucessfully default the LUT to 0d16384 = 0b0100000000000000 = 1 for a fixed point Fx2.14
	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchan, 1.0);
}

void PockelsCell::pushVoltageSinglet(const double timeStep, const double AO, const OVERRIDE override) const
{
	if (AO < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The control voltage of the Pockls cell must be positive");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchan, timeStep, AO, override);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P, const OVERRIDE override) const
{
	if (P < 0 || P > mMaxPower)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power of the pockels cell must be in the range [0-" + std::to_string(static_cast<int>(mMaxPower / mW)) + "] mW");

	mRTcontrol.pushAnalogSinglet(mPockelsRTchan, timeStep, laserpowerToVolt_(P), override);
}

void PockelsCell::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mPockelsRTchan, AO_tMIN, 0 * V);
}

//Linearly scale the pockels voltage from the first to the last frame
void PockelsCell::voltageLinearScaling(const double Vi, const double Vf) const
{
	if (mRTcontrol.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	//Make sure that Fx2p14 will not overflow
	const double Vratio{ Vf / Vi };
	if (Vratio > 4)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested scaling factor must be in the range [0-4]");

	pushVoltageSinglet(timeStep, Vi, OVERRIDE::EN);	//Set the laser power for the first frame

	//Delete the default scaling factors = 1.0 created in the PockelsCell constructor
	mRTcontrol.clearQueue(mScalingRTchan);

	//Push the scaling factors
	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchan, 1 + (Vratio - 1) / (mRTcontrol.mNframes - 1) * ii);
}

//Linearly scale the laser power from the first to the last frame
void PockelsCell::powerLinearScaling(const double Pi, const double Pf) const
{
	if (mRTcontrol.mNframes < 2)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	//Delete the default scaling factors = 1.0 created in the PockelsCell constructor
	mRTcontrol.clearQueue(mScalingRTchan);

	const double Vi{ laserpowerToVolt_(Pi) };
	pushVoltageSinglet(timeStep, Vi, OVERRIDE::EN);	//Set the laser power for the first frame

	for (int ii = 0; ii < mRTcontrol.mNframes; ii++)
	{
		const double V{ laserpowerToVolt_(Pi + (Pf - Pi) / (mRTcontrol.mNframes - 1) * ii) };
		const double Vratio{ V / Vi };

		//Make sure that Fx2p14 will not overflow
		if (Vratio > 4)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested scaling factor must be in the range [0-4]");
		
		//Push the scaling factors for the frames
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchan, 1 + (Vratio - 1) / (mRTcontrol.mNframes - 1) * ii);
	}
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

	mRTcontrol.pushLinearRamp(mPockelsRTchan, timeStep, rampDuration, Vi, Vf);
}

//Ramp up or down the pockels cell within a frame. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void  PockelsCell::powerLinearRampInFrame(const double timeStep, const double rampDuration, const double Pi, const double Pf) const
{
	if (Pi < 0 || Pf < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell's control voltage must be positive");

	mRTcontrol.pushLinearRamp(mPockelsRTchan, timeStep, rampDuration, laserpowerToVolt_(Pi), laserpowerToVolt_(Pf));
}
*/

double PockelsCell::laserpowerToVolt_(const double power) const
{
	double amplitude, angularFreq, phase;		//Calibration parameters

	//VISION
	switch (mPockelsRTchan)
	{
	case RTcontrol::RTCHAN::VISION:
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
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		}			
		break;

		//FIDELITY
	case RTcontrol::RTCHAN::FIDELITY:
		amplitude = 1000 * mW;
		angularFreq = 0.276 / V;
		phase = -0.049 * V;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	double arg{ sqrt(power / amplitude) };
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The argument of asin must be <= 1");

	return asin(arg) / angularFreq + phase;
}
#pragma endregion "Pockels cells"

#pragma region "StepperActuator"
StepperActuator::StepperActuator(const char* serialNumber) : mSerialNumber{ serialNumber }
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

	//Start the device polling at 200ms intervals
	SCC_StartPolling(mSerialNumber, 200);
	
	//Set the actuator velocity
	Sleep(100);
	SCC_SetVelParams(mSerialNumber, mAcc_iu, mVel_iu);

	//download the current position
	mPosition = SCC_GetPosition(mSerialNumber) / mCalib;
	//std::cout << "Collector lens position: " << mPosition / mm << " mm\n";
}

StepperActuator::~StepperActuator()
{
	//Stop polling
	SCC_StopPolling(mSerialNumber);
	SCC_Close(mSerialNumber);	//Close device
}

void StepperActuator::move(const double position)
{
	if (position < mPosLimit.at(0) || position > mPosLimit.at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested position for the collector lens must be in the range [0-13] mm");

	//Move to the new position if different from the current position (within an error)
	if (std::abs(position - mPosition) > 0.001 * mm)
	{
		//Move to position
		SCC_ClearMessageQueue(mSerialNumber);
		SCC_MoveToPosition(mSerialNumber, static_cast<int>(position * mCalib));

		//Thread-safe message
		std::stringstream msg;
		msg << "Positioning the collector lens at " << position / mm << " mm...\n";
		std::cout << msg.str();

		//Wait for completion
		WORD messageType, messageId;
		DWORD messageData;
		SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
		while (messageType != 2 || messageId != 1)
		{
			SCC_WaitForMessage(mSerialNumber, &messageType, &messageId, &messageData);
		}
		//For debugging. Get current position
		//std::cout << "Collector lens current position: " << SCC_GetPosition(mSerialNumber) / mCalib /mm << " mm\n";

		//Update the current position
		mPosition = position;
	}
}

void StepperActuator::downloadConfig() const
{
	Sleep(100);	//The code does not work without this sleep
	std::cout << "Collector lens position: " << SCC_GetPosition(mSerialNumber) / mCalib / mm << " mm\n";

	int currentVelocity, currentAcceleration;
	SCC_GetVelParams(mSerialNumber, &currentAcceleration, &currentVelocity);
	std::cout << "Collector lens acceleration: " << currentAcceleration << " iu\tvelocity: " << currentVelocity << " iu\n";
}

void StepperActuator::home()
{
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
	//Update the current position
	mPosition = 0;
}
#pragma endregion "StepperActuator"


//Integrate the lasers, pockels cells, and filterwheels in a single class
#pragma region "VirtualLaser"

void VirtualLaser::CollectorLens::move(const double position)
{
	mStepper.move(position);
}

void VirtualLaser::CollectorLens::set(const int wavelength_nm)
{
	switch (wavelength_nm)
	{
	case 750:
		mStepper.move(cLensPos750nm);
		break;
	case 920:
		mStepper.move(cLensPos920nm);
		break;
	case 1040:
		mStepper.move(cLensPos1040nm);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The collector lens position has not been calibrated for the wavelength " + std::to_string(wavelength_nm));
	}
}

#pragma region "VirtualFilterWheel"
VirtualLaser::VirtualFilterWheel::VirtualFilterWheel() : mFWexcitation{ Filterwheel::ID::EXC }, mFWdetection{ Filterwheel::ID::DET } {}

void VirtualLaser::VirtualFilterWheel::turnFilterwheels_(const int wavelength_nm)
{
	if (multibeam)//Multiplex. Turn both filterwheels concurrently
	{
		std::future<void> th1{ std::async(&Filterwheel::setWavelength, &mFWexcitation, wavelength_nm) };
		std::future<void> th2{ std::async(&Filterwheel::setWavelength, &mFWdetection, wavelength_nm) };

		try
		{
		th1.get();
		th2.get();
		}
		catch (...)
		{
			throw;
		}
	}

	else //Single beam. Turn both filterwheels concurrently
	{
		std::future<void> th1{ std::async(&Filterwheel::setColor, &mFWexcitation, Filterwheel::COLOR::OPEN) };
		std::future<void> th2{ std::async(&Filterwheel::setWavelength, &mFWdetection, wavelength_nm) };

		try
		{
			th1.get();
			th2.get();
		}
		catch (...)
		{
			throw;
		}
	}
}
#pragma endregion "VirtualFilterWheel"

#pragma region "CombinedLasers"
VirtualLaser::CombinedLasers::CombinedLasers(RTcontrol &RTcontrol, const Laser::ID whichLaser) : mRTcontrol{ RTcontrol }, mWhichLaser{ whichLaser }, mVision{ Laser::ID::VISION }, mFidelity{ Laser::ID::FIDELITY } {}

//Which laser is currently being used
Laser::ID VirtualLaser::CombinedLasers::currentLaser() const
{
	return mCurrentLaser;
}

std::string VirtualLaser::CombinedLasers::currentLaser_s(const bool justTheNameInitials) const
{
	std::string fullName{ laserNameToString_(mCurrentLaser) };
	std::string nameInitials{ fullName.front() };

	if (justTheNameInitials && nameInitials.size() != 0)
		return nameInitials;

	return fullName;
}

int VirtualLaser::CombinedLasers::currentWavelength_nm() const
{
	switch (mCurrentLaser)
	{
	case Laser::ID::VISION:
		return mVision.currentWavelength_nm();
	case Laser::ID::FIDELITY:
		return mFidelity.currentWavelength_nm();
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

void VirtualLaser::CombinedLasers::isLaserInternalShutterOpen() const
{
	while (true)
	{
		bool isShutterOpen;

		//Check which laser is being used
		switch (mCurrentLaser)
		{
		case Laser::ID::VISION:
			isShutterOpen = mVision.isShutterOpen();
			break;
		case Laser::ID::FIDELITY:
			isShutterOpen = mFidelity.isShutterOpen();
			break;
		}//switch

		//Check if the corresponding internal shutter is open
		if (!isShutterOpen)
		{
			std::cout << "The internal shutter of " + laserNameToString_(mCurrentLaser) + " seems to be closed. Press ESC to exit or any other key to try again\n";

			if (_getch() == 27)
				throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		}
		else
			break; //break the whileloop
	}//whileloop
}

//Tune the laser wavelength (for VISION only)
void VirtualLaser::CombinedLasers::tuneLaserWavelength(const int wavelength_nm)
{
	//Select the laser to be used: VISION or FIDELITY
	mCurrentLaser = autoSelectLaser_(wavelength_nm);

	std::stringstream msg;
	msg << "Using " << laserNameToString_(mCurrentLaser) << " at " << wavelength_nm << " nm\n";
	std::cout << msg.str();

	//If VISION is selected, set the new wavelength
	if (mCurrentLaser == Laser::ID::VISION)
		mVision.setWavelength(wavelength_nm);

	//Update the pockels handler to initialize or update the laser power
	//The pockels destructor is made to close the uniblitz shutter automatically for switching from VISION to FIDELITY or viceversa without photobleaching the sample, and also for tuning VISION's wavelength
	mPockelsPtr.reset(new PockelsCell(mRTcontrol, wavelength_nm, mCurrentLaser));
}

void VirtualLaser::CombinedLasers::setPower(const double initialPower, const double finalPower) const
{
	//Set the initial laser power
	mPockelsPtr->pushPowerSinglet(mPockelTimeStep, initialPower, OVERRIDE::EN);

	//Linearly scale the laser power across the frames
	if (finalPower != initialPower)
		mPockelsPtr->powerLinearScaling(initialPower, finalPower);
}

//Linearly scale the laser power from the first to the last frame
void VirtualLaser::CombinedLasers::powerLinearScaling(const double Pi, const double Pf) const
{
	mPockelsPtr->powerLinearScaling(Pi, Pf);
}

void VirtualLaser::CombinedLasers::openShutter() const
{
	mPockelsPtr->setShutter(true);
}

void VirtualLaser::CombinedLasers::closeShutter() const
{
	mPockelsPtr->setShutter(false);
}

std::string VirtualLaser::CombinedLasers::laserNameToString_(const Laser::ID whichLaser) const
{
	switch (whichLaser)
	{
	case Laser::ID::VISION:
		return "VISION";
	case Laser::ID::FIDELITY:
		return "FIDELITY";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser unavailable");
	}
}

//Automatically select a laser: VISION, FIDELITY, or let the code to decide based on the requested wavelength
Laser::ID VirtualLaser::CombinedLasers::autoSelectLaser_(const int wavelength_nm) const
{
	//Use VISION for everything below 1040 nm. Use FIDELITY for 1040 nm	
	if (mWhichLaser == Laser::ID::AUTO)
	{
		if (wavelength_nm < 1040)
			return Laser::ID::VISION;
		else if (wavelength_nm == 1040)
			return Laser::ID::FIDELITY;
		else
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Wavelengths > 1040 nm is not implemented in the CombinedLasers class");
	}
	else //If mWhichLaser != ID::AUTO, the mWhichLaser is either ID::VISION or ID::FIDELITY
		return mWhichLaser;
}
#pragma endregion "CombinedLasers"

VirtualLaser::VirtualLaser(RTcontrol &RTcontrol, const int wavelength_nm, const double initialPower, const double finalPower, const Laser::ID whichLaser) : mCombinedLasers{ RTcontrol, whichLaser }
{
	//Tune the laser wavelength, set the excitation and emission filterwheels, and position the collector lens concurrently
	configure(wavelength_nm);		

	//Set the laser power
	setPower(initialPower, finalPower);
}

VirtualLaser::VirtualLaser(RTcontrol &RTcontrol, const int wavelength_nm, const Laser::ID whichLaser) : mCombinedLasers{ RTcontrol, whichLaser }
{
	//Tune the laser wavelength, set the excitation and emission filterwheels, and position the collector lens concurrently
	configure(wavelength_nm);
}

//This constructor requires to call VirtualLaser::configure() and VirtualLaser::setPower() manually, otherwise some class members will no be initialized properly
VirtualLaser::VirtualLaser(RTcontrol &RTcontrol, const Laser::ID whichLaser) : mCombinedLasers{ RTcontrol, whichLaser } {}

//Which laser is currently being used
Laser::ID VirtualLaser::currentLaser() const
{
	return mCombinedLasers.currentLaser();
}

std::string VirtualLaser::currentLaser_s(const bool justTheNameInitials) const
{
	return mCombinedLasers.currentLaser_s(justTheNameInitials);
}

int VirtualLaser::currentWavelength_nm() const
{
	return mCombinedLasers.currentWavelength_nm();
}

//Tune the laser wavelength, set the exc and emission filterwheels, and position the collector lens
void VirtualLaser::configure(const int wavelength_nm)
{
	std::future<void> th1{ std::async(&CombinedLasers::tuneLaserWavelength, &mCombinedLasers, wavelength_nm) };			//Tune the laser wavelength
	std::future<void> th2{ std::async(&VirtualFilterWheel::turnFilterwheels_, &mVirtualFilterWheel, wavelength_nm) };	//Set the filterwheels
	std::future<void> th3{ std::async(&CollectorLens::set, &mCollectorLens, wavelength_nm) };							//Set the collector lens position

	try
	{
		th1.get();
		th2.get();
		th3.get();
	}
	catch (...)
	{
		throw;
	}

	//Check if the laser internal shutter is open
	mCombinedLasers.isLaserInternalShutterOpen();
}

void VirtualLaser::setPower(const double laserPower) const
{
	mCombinedLasers.setPower(laserPower, laserPower);
}

void VirtualLaser::setPower(const double initialPower, const double finalPower) const
{
	mCombinedLasers.setPower(initialPower, finalPower);
}

//Linearly scale the laser power from the first to the last frame
void VirtualLaser::powerLinearScaling(const double Pi, const double Pf) const
{
	mCombinedLasers.powerLinearScaling(Pi, Pf);
}

void VirtualLaser::openShutter() const
{
	mCombinedLasers.openShutter();
}

void VirtualLaser::closeShutter() const
{
	mCombinedLasers.closeShutter();
}

//Used for optimizing the collector lens position
void VirtualLaser::moveCollectorLens(const double position)
{
	mCollectorLens.move(position);
}
#pragma endregion "VirtualLaser"

#pragma region "Galvo"
Galvo::Galvo(RTcontrol &RTcontrol, const RTcontrol::RTCHAN whichGalvo, const double posMax, const VirtualLaser *virtualLaser) : mRTcontrol{ RTcontrol }, mWhichGalvo{ whichGalvo }, mPosMax{ posMax }
{
	reconfigure(virtualLaser);
}

void Galvo::reconfigure(const VirtualLaser *virtualLaser)
{
	Laser::ID whichLaser;
	int wavelength_nm;

	//Make sure that the laser and wavelength are well defined in virtualLaser, otherwise use Vision at 750 nm as default
	if (virtualLaser != nullptr)
	{
		whichLaser = virtualLaser->currentLaser();
		wavelength_nm = virtualLaser->currentWavelength_nm();
	}
	else
	{
		whichLaser = Laser::ID::VISION;
		wavelength_nm = 750;
	}

	//Choose the scan or rescan galvo because the calibration parameters are different
	switch (mWhichGalvo)//which GALVO
	{
	case RTcontrol::RTCHAN::SCANGALVO:
		mVoltagePerDistance = scannerCalib.voltagePerDistance;
		mVoltageOffset = scannerCalib.voltageOffset;

		//For debugging
		//std::cout << "Scanner mVoltagePerDistance = " << mVoltagePerDistance << "\n";
		//std::cout << "Scanner mVoltageOffset = " << mVoltageOffset << "\n";

		//Raster scan the sample from the positive to the negative direction of the x-stage
		positionLinearRamp(-mPosMax, mPosMax, mVoltageOffset, OVERRIDE::EN);
		break;
	case RTcontrol::RTCHAN::RESCANGALVO:

		//The calibration of the rescanner is slightly different when using Vision or Fidelity
		switch (whichLaser)
		{

		//The calibration of the rescanner also depends on the wavelength used
		case Laser::ID::VISION:
			switch (wavelength_nm)
			{
			case 750:
				mVoltagePerDistance = rescannerCalibV750nm.voltagePerDistance;
				mVoltageOffset = rescannerCalibV750nm.voltageOffset;
				break;
			case 920:
				mVoltagePerDistance = rescannerCalibV920nm.voltagePerDistance;
				mVoltageOffset = rescannerCalibV920nm.voltageOffset;
				break;
			case 1040:
				mVoltagePerDistance = rescannerCalibV1040nm.voltagePerDistance;
				mVoltageOffset = rescannerCalibV1040nm.voltageOffset;
				break;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": The galvo has not been calibrated for the wavelength " + std::to_string(wavelength_nm) + " nm");
			}
			break;
		case Laser::ID::FIDELITY:
			switch (wavelength_nm)
			{
			case 1040:
				mVoltagePerDistance = rescannerCalibF1040nm.voltagePerDistance;
				mVoltageOffset = rescannerCalibF1040nm.voltageOffset;
				break;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": FIDELITY only supports the wavelength 1040 nm\n");
			}
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser unavailable");
		}

		//For debugging
		//std::cout << "Rescanner mVoltagePerDistance = " << mVoltagePerDistance << "\n";
		//std::cout << "Rescanner mVoltageOffset = " << mVoltageOffset << "\n";

		//Rescan in the direction opposite to the scan galvo to keep the fluorescent spot fixed at the detector. If using a single beam (no multiplexing), aim it at a specific channel of the PMT16X
		positionLinearRamp(mPosMax, -mPosMax, mVoltageOffset + beamletIndex_(mRTcontrol.mPMT16Xchan) * mInterBeamletDistance * mVoltagePerDistance,	OVERRIDE::EN);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
	}
}

double Galvo::beamletIndex_(RTcontrol::PMT16XCHAN PMT16Xchan_int) const
{
	switch (PMT16Xchan_int)
	{
	case RTcontrol::PMT16XCHAN::CH00:
		return 7.5;
	case RTcontrol::PMT16XCHAN::CH01:
		return 6.5;
	case RTcontrol::PMT16XCHAN::CH02:
		return 5.5;
	case RTcontrol::PMT16XCHAN::CH03:
		return 4.5;
	case RTcontrol::PMT16XCHAN::CH04:
		return 3.5;
	case RTcontrol::PMT16XCHAN::CH05:
		return 2.5;
	case RTcontrol::PMT16XCHAN::CH06:
		return 1.5;
	case RTcontrol::PMT16XCHAN::CH07:
		return 0.5;
	case RTcontrol::PMT16XCHAN::CH08:
		return -0.5;
	case RTcontrol::PMT16XCHAN::CH09:
		return -1.5;
	case RTcontrol::PMT16XCHAN::CH10:
		return -2.5;
	case RTcontrol::PMT16XCHAN::CH11:
		return -3.5;
	case RTcontrol::PMT16XCHAN::CH12:
		return -4.5;
	case RTcontrol::PMT16XCHAN::CH13:
		return -5.5;
	case RTcontrol::PMT16XCHAN::CH14:
		return -6.5;
	case RTcontrol::PMT16XCHAN::CH15:
		return -7.5;
	case RTcontrol::PMT16XCHAN::CENTERED:
		return 0.0;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected PMT16X channel unavailable");
	}
}

void Galvo::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mWhichGalvo, AO_tMIN, 0 * V);
}

void Galvo::pushVoltageSinglet(const double timeStep, const double AO) const
{
	mRTcontrol.pushAnalogSinglet(mWhichGalvo, timeStep, AO);
}

void Galvo::voltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf, const OVERRIDE override) const
{
	mRTcontrol.pushLinearRamp(mWhichGalvo, timeStep, rampLength, Vi, Vf, override);
}

void Galvo::positionLinearRamp(const double timeStep, const double rampLength, const double posInitial, const double posFinal, const OVERRIDE override) const
{
	mRTcontrol.pushLinearRamp(mWhichGalvo, timeStep, rampLength, mVoltagePerDistance * posInitial, mVoltagePerDistance * posFinal, override);
}

//Generate a linear ramp to scan the galvo across a frame (i.e., in a plane with a fixed z)
void Galvo::positionLinearRamp(const double posInitial, const double posFinal, const double voltageOffset, const OVERRIDE override) const
{
	//Limit the number of steps for long ramps
	//Currently, the bottleneck is the buffer of the galvos on the fpga because it only supports 5000 elements
	//For timeStep = 2 us, the max ramp duration is 10 ms. Therefore, 10 ms/ 62.5 us = 160 lines scanned
	double timeStep;
	if (mRTcontrol.mHeightPerBeamletPerFrame_pix <= 100)
		timeStep = 2. * us;
	else
		timeStep = 8. * us;

	//The position offset allows to compensate for the slight axis misalignment of the rescanner
	mRTcontrol.pushLinearRamp(mWhichGalvo, timeStep, lineclockHalfPeriod * mRTcontrol.mHeightPerBeamletPerFrame_pix + mRampDurationFineTuning,
		voltageOffset + mVoltagePerDistance * posInitial, voltageOffset + mVoltagePerDistance * posFinal, override);
}
#pragma endregion "Galvo"

#pragma region "Stages"
Stage::Stage(const double velX, const double velY, const double velZ, const std::vector<double2> stageSoftPosLimXYZ) : mSoftPosLimXYZ{ stageSoftPosLimXYZ }
{
	const std::string stageIDx{ "116049107" };	//X-stage (V-551.4B)
	const std::string stageIDy{ "116049105" };	//Y-stage (V-551.2B)
	const std::string stageIDz{ "0165500631" };	//Z-stage (ES-100)

	//Open the connections to the stage controllers and assign the IDs
	std::cout << "Establishing connection with the stages\n";
	mID_XYZ.at(X) = PI_ConnectUSB(stageIDx.c_str());
	mID_XYZ.at(Y) = PI_ConnectUSB(stageIDy.c_str());
	mID_XYZ.at(Z) = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID_XYZ.at(Z) = PI_ConnectUSB(stageIDz.c_str());

	if (mID_XYZ.at(X) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");

	if (mID_XYZ.at(Y) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");

	if (mID_XYZ.at(Z) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	std::cout << "Connection with the stages successfully established\n";

	//Download the current position
	mPositionXYZ.at(X) = downloadPositionSingle_(X);
	mPositionXYZ.at(Y) = downloadPositionSingle_(Y);
	mPositionXYZ.at(Z) = downloadPositionSingle_(Z);

	//Download the current velocities
	mVelXYZ.at(X) = downloadVelSingle_(X);
	mVelXYZ.at(Y) = downloadVelSingle_(Y);
	mVelXYZ.at(Z) = downloadVelSingle_(Z);

	configDOtriggers_();				//Configure the stage velocities and DO triggers
	setVelXYZ({ velX, velY, velZ });	//Set the stage velocities
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mID_XYZ.at(X));
	PI_CloseConnection(mID_XYZ.at(Y));
	PI_CloseConnection(mID_XYZ.at(Z));
	//std::cout << "Connection with the stages successfully closed\n";
}

//Recall the current position for the 3 stages
double3 Stage::readPositionXYZ() const
{
	return mPositionXYZ;
}

void Stage::printPositionXYZ() const
{
	std::cout << "Stage X position = " << mPositionXYZ.at(X) / mm << " mm\n";
	std::cout << "Stage Y position = " << mPositionXYZ.at(Y) / mm << " mm\n";
	std::cout << "Stage Z position = " << mPositionXYZ.at(Z) / mm << " mm\n";
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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested position is out of the soft limits of the stage " + axisToString(axis));
	if (position < mTravelRangeXYZ.at(axis).at(0) || position > mTravelRangeXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested position is out of the physical limits of the stage " + axisToString(axis));

	//Move the stage
	if (mPositionXYZ.at(axis) != position) //Move only if the requested position is different from the current position
	{
		const double position_mm{ position / mm };								//Divide by mm to convert from implicit to explicit units
		if (!PI_MOV(mID_XYZ.at(axis), mNstagesPerController, &position_mm))		//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage " + axisToString(axis) + " to the target position (maybe hardware limits?)");

		mPositionXYZ.at(axis) = position;
	}
}

//Move the 3 stages to the requested position
void Stage::moveXY(const double2 positionXY)
{
	moveSingle(X, positionXY.at(X));
	moveSingle(Y, positionXY.at(Y));
}

//Move the 3 stages to the requested position
void Stage::moveXYZ(const double3 positionXYZ)
{
	moveSingle(X, positionXYZ.at(X));
	moveSingle(Y, positionXYZ.at(Y));
	moveSingle(Z, positionXYZ.at(Z));
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
	std::cout << "Stage " + axisToString(axis) + " moving to the new position: ";

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
		if (!PI_IsMoving(mID_XYZ.at(X), mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mID_XYZ.at(Y), mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mID_XYZ.at(Z), mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << ".";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}

void Stage::stopAll() const
{
	PI_StopAll(mID_XYZ.at(X));
	PI_StopAll(mID_XYZ.at(Y));
	PI_StopAll(mID_XYZ.at(Z));

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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The velocity must be >0 for the stage " + axisToString(axis));

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
	setVelSingle(X, velXYZ.at(X));
	setVelSingle(Y, velXYZ.at(Y));
	setVelSingle(Z, velXYZ.at(Z));
}

void Stage::printVelXYZ() const
{
	std::cout << "Stage X vel = " << mVelXYZ.at(X) / mmps << " mm/s\n";
	std::cout << "Stage Y vel = " << mVelXYZ.at(Y) / mmps << " mm/s\n";
	std::cout << "Stage Z vel = " << mVelXYZ.at(Z) / mmps << " mm/s\n";
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
double Stage::downloadDOtriggerParamSingle_(const Axis axis, const DIOCHAN DOchan, const DOPARAM triggerParamID) const
{
	const int triggerParamID_int{ static_cast<int>(triggerParamID) };
	const int DOchan_int{ static_cast<int>(DOchan) };
	double value;
	if (!PI_qCTO(mID_XYZ.at(axis), &DOchan_int, &triggerParamID_int, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger config for the stage " + axisToString(axis));

	//std::cout << value << "\n";
	return value;
}

void Stage::setDOtriggerParamSingle(const Axis axis, const DIOCHAN DOchan, const DOPARAM triggerParamID, const double value) const
{
	const int triggerParamID_int{ static_cast<int>(triggerParamID) };
	const int DOchan_int{ static_cast<int>(DOchan) };
	if (!PI_CTO(mID_XYZ.at(axis), &DOchan_int, &triggerParamID_int, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger config for the stage " + axisToString(axis));
}

void Stage::setDOtriggerParamAll(const Axis axis, const DIOCHAN DOchan, const double triggerStep, const DOTRIGMODE triggerMode, const double startThreshold, const double stopThreshold) const
{
	if (triggerStep <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The trigger step must be >0");

	if (startThreshold < mTravelRangeXYZ.at(axis).at(0) || startThreshold > mTravelRangeXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The start position is out of the physical limits of the stage " + axisToString(axis));

	if (stopThreshold < mTravelRangeXYZ.at(axis).at(0) || stopThreshold > mTravelRangeXYZ.at(axis).at(1))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stop position is out of the physical limits of the stage " + axisToString(axis));


	setDOtriggerParamSingle(axis, DOchan, DOPARAM::TRIGSTEP, triggerStep / mm);					//Trigger step
	setDOtriggerParamSingle(axis, DOchan, DOPARAM::AXISNUMBER, 1);								//Axis of the controller (always 1 because each controller has only 1 stage)
	setDOtriggerParamSingle(axis, DOchan, DOPARAM::TRIGMODE, static_cast<double>(triggerMode));	//Trigger mode
	setDOtriggerParamSingle(axis, DOchan, DOPARAM::POLARITY, 1);								//POLARITY (0 for active low, 1 for active high)
	setDOtriggerParamSingle(axis, DOchan, DOPARAM::STARTTHRES, startThreshold / mm);			//Start threshold
	setDOtriggerParamSingle(axis, DOchan, DOPARAM::STOPTHRES, stopThreshold / mm);				//Stop threshold
}

//Request the enable/disable status of the stage DO
bool Stage::isDOtriggerEnabled(const Axis axis, const DIOCHAN DOchan) const
{
	BOOL triggerState;
	const int DOchan_int{ static_cast<int>(DOchan) };
	if (!PI_qTRO(mID_XYZ.at(axis), &DOchan_int, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger EN/DIS stage for the stage " + axisToString(axis));

	//std::cout << triggerState << "\n";
	return triggerState;
}

//Enable or disable the stage DO
void Stage::setDOtriggerEnabled(const Axis axis, const DIOCHAN DOchan, const BOOL triggerState) const
{
	const int DOchan_int{ static_cast<int>(DOchan) };
	if (!PI_TRO(mID_XYZ.at(axis), &DOchan_int, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger EN/DIS state for the stage " + axisToString(axis));
}

//Each stage driver has 4 DO channels that can be used to monitor the stage position, motion, etc
//Print out the relevant parameters
void Stage::printStageConfig(const Axis axis, const DIOCHAN chan) const
{
	switch (axis)
	{
	case X:
		//Only DO1 is wired to the FPGA
		if (chan != DIOCHAN::D1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	case Y:
		//Only DO1 is wired to the FPGA
		if (chan != DIOCHAN::D1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	case Z:
		//Only DO1 and DO2 are wired to the FPGA
		if (chan != DIOCHAN::D1 || chan != DIOCHAN::D2)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 and DO2 are currently wired to the FPGA for the stage " + axisToString(axis));
		break;
	}

	const double triggerStep_mm{ downloadDOtriggerParamSingle_(axis, chan, DOPARAM::TRIGSTEP) };
	const int triggerMode{ static_cast<int>(downloadDOtriggerParamSingle_(axis, chan, DOPARAM::TRIGMODE)) };
	const int polarity{ static_cast<int>(downloadDOtriggerParamSingle_(axis, chan, DOPARAM::POLARITY)) };
	const double startThreshold_mm{ downloadDOtriggerParamSingle_(axis, chan, DOPARAM::STARTTHRES) };
	const double stopThreshold_mm{ downloadDOtriggerParamSingle_(axis, chan, DOPARAM::STOPTHRES) };
	const double triggerPosition_mm{ downloadDOtriggerParamSingle_(axis, chan, DOPARAM::TRIGPOS) };
	const bool triggerState{ isDOtriggerEnabled(axis, chan) };
	const double vel{ downloadVelSingle_(axis) };

	std::cout << "Configuration for the stage = " << axisToString(axis) << ", DOchan = " << static_cast<int>(chan) << ":\n";
	std::cout << "is DO trigger enabled? = " << triggerState << "\n";
	std::cout << "Trigger step = " << triggerStep_mm << " mm\n";
	std::cout << "Trigger mode = " << triggerMode << "\n";
	std::cout << "POLARITY = " << polarity << "\n";
	std::cout << "Start threshold position = " << startThreshold_mm << " mm\n";
	std::cout << "Stop threshold position = " << stopThreshold_mm << " mm\n";
	std::cout << "Trigger position = " << triggerPosition_mm << " mm\n";
	std::cout << "Vel = " << vel / mmps << " mm/s\n\n";
}

//DO1 and DO2 of STAGEZ are used to trigger the stack acquisition. Currently only DO2 is used as trigger. See the implementation on LV
void Stage::configDOtriggers_() const
{

	//STAGE Y. DO1 (PIN5 of the RS232 connector). DI2 (PIN2 of the RS232 connector). AFAIR, DI1 was not working properly as an an input pin (disabled by the firmware?)
	const DIOCHAN YDO1{ DIOCHAN::D1 };
	setDOtriggerEnabled(Y, YDO1, true);																//Enable DO1 output
	setDOtriggerParamSingle(Y, YDO1, DOPARAM::TRIGMODE, static_cast<double>(DOTRIGMODE::INMOTION));	//Configure DO1 as motion monitor


	//STAGE Z
	//DO1 TRIGGER: DO1 is set to output a pulse (fixed width = 50 us) whenever the stage covers a certain distance (e.g. 0.3 um)
	/*
	const int DO1{ 1 };
	setDOtriggerEnabled(Z, DO1, true);	//Enable DO1 trigger
	const double triggerStep{ 0.3 * um };
	const DOTRIGMODE triggerMode{ POSDIST };
	const double startThreshold{ 0. * mm };
	const double stopThreshold{ 0. * mm };
	setDOtriggerParamAll(Z, DO1, triggerStep, triggerMode, startThreshold, stopThreshold);
	*/

	//DO2 TRIGGER: DO2 is set to output HIGH when the stage z is in motion
	const DIOCHAN ZDO2{ DIOCHAN::D2 };
	setDOtriggerEnabled(Z, ZDO2, true);																//Enable DO2 output
	setDOtriggerParamSingle(Z, ZDO2, DOPARAM::TRIGMODE, static_cast<double>(DOTRIGMODE::INMOTION));	//Configure DO2 as motion monitor
}

std::string Stage::axisToString(const Axis axis) const
{
	switch (axis)
	{
	case X:
		return "X";
	case Y:
		return "Y";
	case Z:
		return "Z";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid stage axis");
	}
}
#pragma endregion "Stages"

#pragma region "Vibratome"
Vibratome::Vibratome(const FPGA &fpga, Stage &stage) : mFpga{ fpga }, mStage{ stage } {}

//Start or stop running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::pushStartStopButton() const
{
	const int pulsewidth{ 100 * ms }; //in ms. It has to be longer than~ 12 ms, otherwise the vibratome is not triggered

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_VTstart, true));

	Sleep(static_cast<DWORD>(pulsewidth / ms));

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
}

void Vibratome::slice(const double planeToCutZ)
{
	mStage.setVelXYZ(mStageConveyingVelXYZ);																		//Change the velocity to move the sample to the vibratome
	mStage.moveXYZ({ mStageInitialSlicePosXY.at(Stage::X), mStageInitialSlicePosXY.at(Stage::Y), planeToCutZ });	//Position the sample in front of the vibratome's blade
	mStage.waitForMotionToStopAll();

	mStage.setVelSingle(Stage::Y, mSlicingVel);								//Change the y vel for slicing
	pushStartStopButton();													//Turn on the vibratome
	mStage.moveSingle(Stage::Y, mStageFinalSlicePosY);						//Slice the sample: move the stage y towards the blade
	mStage.waitForMotionToStopSingle(Stage::Y);								//Wait until the motion ends
	mStage.setVelSingle(Stage::Y, mStageConveyingVelXYZ.at(Stage::Y));		//Set back the y vel to move the sample back to the microscope

	//mStage.moveSingle(Y, mStage.mTravelRangeXYZ.at(Y).at(1));				//Move the stage y all the way to the end to push the cutoff slice forward, in case it gets stuck on the sample
	//mStage.waitForMotionToStopSingle(Y);									//Wait until the motion ends

	pushStartStopButton();													//Turn off the vibratome
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

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), selectedChannel, true));

	if (duration >= minDuration)
		Sleep(static_cast<DWORD>((duration - delay)/ms));
	else
	{
		Sleep(static_cast<DWORD>((minDuration - delay)/ms));
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << 1. * minDuration / ms << "ms" << "\n";
	}
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), selectedChannel, false));
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

/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/