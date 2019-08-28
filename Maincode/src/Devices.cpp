#include "Devices.h"

#pragma region "Image"
//When multiplexing, create a mTiff to store 16 strips of height 'mRTcontrol.mHeightPerFrame_pix' each
Image::Image(const RTcontrol &RTcontrol) :
	mRTcontrol{ RTcontrol },
	mTiff{ mRTcontrol.mWidthPerFrame_pix, (static_cast<int>(multibeam) * (g_nChanPMT - 1) + 1) *  mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes }
{}

Image::~Image()
{
	//std::cout << "Image destructor called\n"; //For debugging
}

//Access the Tiff data in the Image object
U8* const Image::data() const
{
	return mTiff.data();
}

//Demultiplex the image
void Image::acquire(const bool saveAllPMT)
{
	demultiplex_(saveAllPMT);	//Copy the chuncks of data to mTiff
	mTiff.mirrorOddFrames();	//The galvo (vectical axis of the image) performs bi-directional scanning frame after frame. Mirror the odd frames vertically
}

//To perform continuous scan in x. Different from Image::acquire() because
//each frame has mHeightPerFrame_pix = 2 (2 swings of the RS) and mNframes = half the pixel height of the final image
void Image::acquireVerticalStrip(const SCANDIR scanDirX)
{
	const bool saveAllPMT{ false };

	demultiplex_(saveAllPMT);	//Copy the chuncks of data to mTiff
	mTiff.mergeFrames();		//Set mNframes = 1 to treat mArray as a single image	

	//Mirror the entire image if a reversed scan was performed
	switch (scanDirX)
	{
	case SCANDIR::RIGHTWARD:
		mTiff.mirror();			
		break;
	}
}

//Image post processing
void Image::correct(const double FFOVfast)
{
	mTiff.correctRSdistortionGPU(FFOVfast);		//Correct the image distortion induced by the nonlinear scanning of the RS

	if (multibeam)
	{
		mTiff.flattenField(2.0, 4, 11);
		mTiff.suppressCrosstalk(0.2);
	}
}

//Correct the image distortion induced by the nonlinear scanning of the RS
void Image::correctRSdistortion(const double FFOVfast)
{
	mTiff.correctRSdistortionGPU(FFOVfast);		
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

//Save each frame in mTiff in either a single Tiff page or different Tiff pages
void Image::save(std::string filename, const TIFFSTRUCT pageStructure, const OVERRIDE override) const
{
	mTiff.saveToFile(filename, pageStructure, override, mRTcontrol.mScanDir);
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
	//Shift bufferA and  bufferB to the right a number of bits depending on the PMT channel to be read
	//For bufferA, shift 0 bits for CH00, 4 bits for CH01, 8 bits for CH02, etc...
	//For mMultiplexedArrayAB, shift 0 bits for CH08, 4 bits for CH09, 8 bits for CH10, etc...
	const unsigned int nBitsToShift{ 4 * static_cast<unsigned int>(mRTcontrol.mPMT16Xchan) };

	//Demultiplex bufferA (CH00-CH07). Each U32 element in bufferA has the multiplexed structure | CH07 (MSB) | CH06 | CH05 | CH04 | CH03 | CH02 | CH01 | CH00 (LSB) |
	if (mRTcontrol.mPMT16Xchan >= RTcontrol::PMT16XCHAN::CH00 && mRTcontrol.mPMT16Xchan <= RTcontrol::PMT16XCHAN::CH07)
	{
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixPerBeamletAllFrames; pixIndex++)
		{
			const int upscaled{ g_upscalingFactor * (((mRTcontrol.dataBufferA())[pixIndex] >> nBitsToShift) & 0x0000000F) };	//Extract the count from the last 4 bits and upscale it to have a 8-bit pixel
			(mTiff.data())[pixIndex] = clipU8top(upscaled);																		//Clip if overflow
		}
	}
	//Demultiplex bufferB (CH08-CH15). Each U32 element in bufferB has the multiplexed structure | CH15 (MSB) | CH14 | CH13 | CH12 | CH11 | CH10 | CH09 | CH08 (LSB) |
	else if (mRTcontrol.mPMT16Xchan >= RTcontrol::PMT16XCHAN::CH08 && mRTcontrol.mPMT16Xchan <= RTcontrol::PMT16XCHAN::CH15)
	{
		for (int pixIndex = 0; pixIndex < mRTcontrol.mNpixPerBeamletAllFrames; pixIndex++)
		{
			const int upscaled{ g_upscalingFactor * (((mRTcontrol.dataBufferB())[pixIndex] >> nBitsToShift) & 0x0000000F) };	//Extract the count from the last 4 bits and upscale it to have a 8-bit pixel
			(mTiff.data())[pixIndex] = clipU8top(upscaled);																		//Clip if overflow
		}
	}
	else
		;//If PMT16XCHAN::CENTERED, do anything
}

//Each U32 element in bufferA and bufferB has the multiplexed structure:
//bufferA[i] =  | CH07 (MSB) | CH06 | CH05 | CH04 | CH03 | CH02 | CH01 | CH00 (LSB) |
//bufferB[i] =  | CH15 (MSB) | CH14 | CH13 | CH12 | CH11 | CH10 | CH09 | CH08 (LSB) |
void Image::demuxAllChannels_(const bool saveAllPMT)
{
	//Use 2 separate arrays to allow parallelization in the future
	TiffU8 CountA{ mRTcontrol.mWidthPerFrame_pix, g_nChanPMT / 2 * mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in CH00-CH07
	TiffU8 CountB{ mRTcontrol.mWidthPerFrame_pix, g_nChanPMT / 2 * mRTcontrol.mHeightPerBeamletPerFrame_pix, mRTcontrol.mNframes };		//Tiff for storing the photocounts in CH08-CH15

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
		for (int chanIndex = 0; chanIndex < g_nChanPMT / 2; chanIndex++)
		{
			//Buffer A (CH00-CH07)
			const int upscaledA{ g_upscalingFactor * ((mRTcontrol.dataBufferA())[pixIndex] & 0x0000000F) };			//Extract the count from the first 4 bits and upscale it to have a 8-bit pixel
			(CountA.data())[chanIndex * mRTcontrol.mNpixPerBeamletAllFrames + pixIndex] = clipU8top(upscaledA);		//Clip if overflow
			(mRTcontrol.dataBufferA())[pixIndex] = (mRTcontrol.dataBufferA())[pixIndex] >> 4;						//Shift 4 places to the right for the next iteration

			//Buffer B (CH08-CH15)
			const int upscaledB{ g_upscalingFactor * ((mRTcontrol.dataBufferB())[pixIndex] & 0x0000000F) };			//Extract the count from the first 4 bits and upscale it to have a 8-bit pixel
			(CountB.data())[chanIndex * mRTcontrol.mNpixPerBeamletAllFrames + pixIndex] = clipU8top(upscaledB);		//Clip if overflow
			(mRTcontrol.dataBufferB())[pixIndex] = (mRTcontrol.dataBufferB())[pixIndex] >> 4;						//Shift 4 places to the right for the next iteration
		}

	//Merge all the PMT16X channels into a single image. The strip ordering depends on the scanning direction of the galvos (forward or backwards)
	if (multibeam)
		mTiff.mergePMT16Xchan(mRTcontrol.mHeightPerBeamletPerFrame_pix, CountA.data(), CountB.data());				//mHeightPerBeamletPerFrame_pix is the height for a single PMT16X channel

	//For debugging
	if (saveAllPMT)
	{
		//Save all PMT16X channels in separate pages in a Tiff
		TiffU8 stack{ mRTcontrol.mWidthPerFrame_pix, mRTcontrol.mHeightPerBeamletPerFrame_pix , g_nChanPMT * mRTcontrol.mNframes };
		stack.pushImage(CountA.data(), static_cast<int>(RTcontrol::PMT16XCHAN::CH00), static_cast<int>(RTcontrol::PMT16XCHAN::CH07));
		stack.pushImage(CountB.data(), static_cast<int>(RTcontrol::PMT16XCHAN::CH08), static_cast<int>(RTcontrol::PMT16XCHAN::CH15));

		std::string PMT16Xchan_s{ std::to_string(static_cast<int>(mRTcontrol.mPMT16Xchan)) };
		stack.saveToFile("AllChannels PMT16Xchan=" + PMT16Xchan_s, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
	}
}
#pragma endregion "Image"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const RTcontrol &RTcontrol) :
	mRTcontrol{ RTcontrol }
{	
	//Calculate the spatial fill factor
	const double temporalFillFactor{ mRTcontrol.mWidthPerFrame_pix * g_pixelDwellTime / g_lineclockHalfPeriod };
	if (temporalFillFactor > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");
	else
		mFillFactor = sin(1. * PI / 2 * temporalFillFactor);			//Note that the fill factor doesn't depend on the RS amplitude
																		//because the RS period is always the same and independent of the amplitude

	//std::cout << "Fill factor = " << mFillFactor << "\n";				//For debugging

	//Download the current control voltage from the FPGA and update the scan parameters
	mControlVoltage = downloadControlVoltage();							//Control voltage
	mFullScan = mControlVoltage / mVoltagePerDistance;					//Full scan FOV = distance between the turning points
	mFFOV = mFullScan * mFillFactor;									//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;					//Spatial sampling resolution (length/pixel)
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
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAfunc::voltageToI16(mControlVoltage)));
}

//First set the FFOV, then set RSenable on
void ResonantScanner::turnOn(const double FFOV)
{
	setFFOV(FFOV);
	Sleep(static_cast<DWORD>(mDelay / ms));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS FFOV successfully set to: " << FFOV / um << " um\n";
}

//First set the control voltage, then set RSenable on
void ResonantScanner::turnOnUsingVoltage(const double controlVoltage)
{
	setVoltage_(controlVoltage);
	Sleep(static_cast<DWORD>(mDelay / ms));
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_ControlBool_RSrun, true));
	std::cout << "RS control voltage successfully set to: " << controlVoltage / V << " V\n";
}

//First set RSenable off, then set the control voltage to 0
void ResonantScanner::turnOff()
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_ControlBool_RSrun, false));
	Sleep(static_cast<DWORD>(mDelay / ms));
	setVoltage_(0);
	std::cout << "RS successfully turned off" << "\n";
}

//Download the current control voltage of the RS from the FPGA
double ResonantScanner::downloadControlVoltage() const
{
	I16 control_I16;
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadI16(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16, &control_I16));

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
		FPGAfunc::checkStatus(__FUNCTION__, NiFpga_ReadBool(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_IndicatorBool_RSisRunning, &isRunning));
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
	mControlVoltage = controlVoltage;							//Control voltage
	mFullScan = controlVoltage / mVoltagePerDistance;			//Full scan FOV
	mFFOV = mFullScan * mFillFactor;							//FFOV
	mSampRes = mFFOV / mRTcontrol.mWidthPerFrame_pix;			//Spatial sampling resolution (length/pixel)

	//Upload the control voltage
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteI16(mRTcontrol.mFpga.handle(), NiFpga_FPGAvi_ControlI16_RSvoltage_I16, FPGAfunc::voltageToI16(mControlVoltage)));
}
#pragma endregion "Resonant scanner"

#pragma region "PMT16X"
PMT16X::PMT16X()	
{
	try
	{
		mSerial = std::unique_ptr<serial::Serial>(new serial::Serial("COM" + std::to_string(static_cast<int>(mPort)), mBaud, serial::Timeout::simpleTimeout(mTimeout / ms)));
	}
	catch (const serial::IOException)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure communicating with the PMT16X");
	}

}

PMT16X::~PMT16X()
{
	mSerial->close();
}

void PMT16X::readAllGains() const
{
	std::vector<uint8_t> parameters{ sendCommand_({'I'}) };

	//The gains are stored in parameters.at(1) to parameters.at(15)
	//Check that the chars returned by the PMT16X are correct. Sum-check the chars till the last two elements in 'parameter', which are the returned sumcheck and CR
	if (parameters.at(0) != 'I' || parameters.at(17) != sumCheck_(parameters, parameters.size() - 2))
		std::cout << "Warning in " + (std::string)__FUNCTION__ + ": CheckSum mismatch\n";

	//Print out the gains
	std::cout << "PMT16X gains:\n";
	for (int ii = 0; ii < g_nChanPMT; ii++)
		std::cout << "Gain CH" << ii << " (0-255) = " << static_cast<int>(parameters.at(ii + 1)) << "\n";
}

void PMT16X::setSingleGain(const RTcontrol::PMT16XCHAN chan, const int gain) const
{
	//Check that the inputVector parameters are within range
	if (chan < RTcontrol::PMT16XCHAN::CH00 || chan > RTcontrol::PMT16XCHAN::CH15)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": PMT16X channel number out of range [1-" + std::to_string(g_nChanPMT) + "]");

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

void PMT16X::setAllGains(const int gain) const
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

void PMT16X::setAllGains(std::vector<uint8_t> gains) const
{
	//Check that the inputVector parameters are within range
	if (gains.size() != g_nChanPMT)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Gain array must have " + std::to_string(g_nChanPMT) + " elements");

	for (int ii = 0; ii < g_nChanPMT; ii++)
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
	for (int ii = 1; ii <= g_nChanPMT; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << static_cast<int>(parameters.at(ii)) << "\n";
}

//Suppress the gain of the middle channels of the PMT16X by suppressFactor. Do a linear interpolation towards the lower and higher channels
void PMT16X::suppressGainsLinearly(const double suppressFactor, const RTcontrol::PMT16XCHAN lowerChan, const RTcontrol::PMT16XCHAN higherChan) const
{
	//Check that the inputVector parameters are within range
	if (suppressFactor < 0 || suppressFactor > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The scale factor must be in the range [0-1.0]");

	if (lowerChan > higherChan)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The lower channel index must be < than the higher channel index");

	const uint8_t gainMax{ 255 };																			//Max gain of the PMT16X channels
	const double gainMin{ suppressFactor * gainMax };														//Gain of the mid PMT16X channels
	const double lowerSlope{ (gainMax - gainMin) / (PMT16XCHANtoInt_(lowerChan)) };							//Interpolation slope for the lower channels
	const double higherSlope{ (gainMax - gainMin) / ((g_nChanPMT - 1) - PMT16XCHANtoInt_(higherChan)) };	//Interpolation slope for the higher channels

	std::vector<uint8_t> gains(g_nChanPMT, static_cast<uint8_t>(std::round(gainMin)));						//Vector of gains

	//Line interpolation lower channles: from CH00 to lowerChan
	for (int chanIndex = 0; chanIndex < PMT16XCHANtoInt_(lowerChan) + 1; chanIndex++)
		gains.at(chanIndex) = static_cast<uint8_t>(std::round( -lowerSlope * chanIndex + gainMax));
	
	//Line interpolation higher channels: from higherChan to CH15
	for (int chanIndex = PMT16XCHANtoInt_(higherChan); chanIndex < g_nChanPMT; chanIndex++)
		gains.at(chanIndex) = static_cast<uint8_t>(std::round( higherSlope * (chanIndex - (g_nChanPMT - 1)) + gainMax));

	//For debugging
	//for (int iterChan = 0; iterChan < g_nChanPMT; iterChan++)
	//	std::cout << "gain " << iterChan << "= " << (int)gains.at(iterChan) << "\n";	

	setAllGains(gains);			//upload the gains to the PMT16X
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

int PMT16X::PMT16XCHANtoInt_(const RTcontrol::PMT16XCHAN chan) const
{
	switch (chan)
	{
	case RTcontrol::PMT16XCHAN::CH00:
		return 0;
	case RTcontrol::PMT16XCHAN::CH01:
		return 1;
	case RTcontrol::PMT16XCHAN::CH02:
		return 2;
	case RTcontrol::PMT16XCHAN::CH03:
		return 3;
	case RTcontrol::PMT16XCHAN::CH04:
		return 4;
	case RTcontrol::PMT16XCHAN::CH05:
		return 5;
	case RTcontrol::PMT16XCHAN::CH06:
		return 6;
	case RTcontrol::PMT16XCHAN::CH07:
		return 7;
	case RTcontrol::PMT16XCHAN::CH08:
		return 8;
	case RTcontrol::PMT16XCHAN::CH09:
		return 9;
	case RTcontrol::PMT16XCHAN::CH10:
		return 10;
	case RTcontrol::PMT16XCHAN::CH11:
		return 11;
	case RTcontrol::PMT16XCHAN::CH12:
		return 12;
	case RTcontrol::PMT16XCHAN::CH13:
		return 13;
	case RTcontrol::PMT16XCHAN::CH14:
		return 14;
	case RTcontrol::PMT16XCHAN::CH15:
		return 15;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected PMT16X channel unavailable");
	}
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
Filterwheel::Filterwheel(const ID whichFilterwheel) :
	mWhichFilterwheel{ whichFilterwheel }
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
				msg << "WARNING: " << mFilterwheelName << " is in the incorrect position " << position << "\n";
				std::cout << msg.str();
				pressAnyKeyToContOrESCtoExit();
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
		return std::stoi(RxBuffer);							//convert string to int
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
Laser::Laser(const ID whichLaser) :
	mWhichLaser{ whichLaser }
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
Shutter::Shutter(const FPGA &fpga, const Laser::ID whichLaser) :
	mFpga{ fpga }
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
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), mWhichShutter, false));
}

//Set the shutter open or close
void Shutter::setShutter(const bool state) const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), mWhichShutter, state));
}

//Open and close the shutter
void Shutter::pulse(const double pulsewidth) const
{
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), mWhichShutter, true));

	Sleep(static_cast<DWORD>(pulsewidth/ms));

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), mWhichShutter, false));
}
#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Curently, the output of the pockels cell is gated on the FPGA side: the output is HIGH when 'framegate' is HIGH
//Each Uniblitz shutter goes with a specific pockels cell, so it makes more sense to control the shutters through the PockelsCell class
PockelsCell::PockelsCell(RTcontrol &RTcontrol, const int wavelength_nm, const Laser::ID laserSelector) :
	mRTcontrol{ RTcontrol },
	mWavelength_nm{ wavelength_nm },
	mShutter{ mRTcontrol.mFpga, laserSelector }
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
	mMaxPower = 1600 * mW;//Multibeam		
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
	mRTcontrol.pushAnalogSinglet(mPockelsRTchan, g_tMinAO, 0 * V);
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
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchan, 1. + (Vratio - 1) / (mRTcontrol.mNframes - 1) * ii);

	//Enable scaling the pockels on the FPGA (see the LV implementation)
	mRTcontrol.enablePockelsScaling();
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
		mRTcontrol.pushAnalogSingletFx2p14(mScalingRTchan, 1. + (Vratio - 1) / (mRTcontrol.mNframes - 1) * ii);
	}

	//Enable scaling the pockels on the FPGA (see the LV implementation)
	mRTcontrol.enablePockelsScaling();
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

// power = powerAmplitude * sin( angularFreq * (V - Vphase))^2 + powerMin;
double PockelsCell::laserpowerToVolt_(const double power) const
{
	double powerAmplitude, powerMin, angularFreq, Vphase;		//Calibration parameters

	//VISION
	switch (mPockelsRTchan)
	{
	case RTcontrol::RTCHAN::VISION:
		switch (mWavelength_nm)
		{
		case 750:
			powerAmplitude = 1600.0 * mW;
			powerMin = 0 * mW;
			angularFreq = 0.624 / V;
			Vphase = 0.019 * V;

			break;
		case 920:
			powerAmplitude = 1089.0 * mW;
			powerMin = 0 * mW;
			angularFreq = 0.507 / V;
			Vphase = -0.088 * V;
			break;
		case 1040:
			powerAmplitude = 388.0 * mW;
			powerMin = 0 * mW;
			angularFreq = 0.447 / V;
			Vphase = 0.038 * V;
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser wavelength " + std::to_string(mWavelength_nm) + " nm has not been calibrated");
		}			
		break;

		//FIDELITY
	case RTcontrol::RTCHAN::FIDELITY:
		powerAmplitude = 1601 * mW;
		powerMin = 23.0 * mW;
		angularFreq = 0.284 / V;
		Vphase = -0.193 * V;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}

	if (power < powerMin)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser min power is " + std::to_string(powerMin) + " mW");

	double arg{ sqrt( (power - powerMin) / powerAmplitude) };
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The argument of asin must be <= 1");

	return asin(arg) / angularFreq + Vphase;
}
#pragma endregion "Pockels cells"

#pragma region "StepperActuator"
StepperActuator::StepperActuator(const char* serialNumber) :
	mSerialNumber{ serialNumber }
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
		mStepper.move(g_cLensPos750nm);
		break;
	case 920:
		mStepper.move(g_cLensPos920nm);
		break;
	case 1040:
		mStepper.move(g_cLensPos1040nm);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The collector lens position has not been calibrated for the wavelength " + std::to_string(wavelength_nm));
	}
}

#pragma region "VirtualFilterWheel"
VirtualLaser::VirtualFilterWheel::VirtualFilterWheel() :
	mFWexcitation{ Filterwheel::ID::EXC },
	mFWdetection{ Filterwheel::ID::DET }
{}

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
VirtualLaser::CombinedLasers::CombinedLasers(const Laser::ID whichLaser) :
	mWhichLaser{ whichLaser },
	mVision{ Laser::ID::VISION },
	mFidelity{ Laser::ID::FIDELITY }
{}

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

//Change the laser wavelength (tune Vision or switching lasers accordingly) and switch pockels
void VirtualLaser::CombinedLasers::setWavelength(RTcontrol &RTcontrol, const int wavelength_nm)
{
	//Select the laser to be used: VISION or FIDELITY
	Laser::ID newLaser = autoSelectLaser_(wavelength_nm);

	std::stringstream msg;
	msg << "Using " << laserNameToString_(newLaser) << " at " << wavelength_nm << " nm\n";
	std::cout << msg.str();

	//For the first call, assign a pointer to mPockelsPtr
	if(mPockelsPtr == nullptr)	
		mPockelsPtr.reset(new PockelsCell(RTcontrol, wavelength_nm, newLaser));
	//For the subsequent calls, destroy the pockels object when switching wavelengths (including switching lasers) to avoid photobleaching the sample (the pockels destructor closes the shutter)
	else if(currentWavelength_nm() != wavelength_nm || mCurrentLaser != newLaser)
		mPockelsPtr.reset(new PockelsCell(RTcontrol, wavelength_nm, newLaser));

	//If VISION is selected, set the new wavelength
	if (newLaser == Laser::ID::VISION)
		mVision.setWavelength(wavelength_nm);

	mCurrentLaser = newLaser;
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

//The constructor established a connection with the 2 lasers.
//VirtualLaser::configure() and VirtualLaser::setPower() must be called after this constructor!! otherwise some class members will no be properly initialized
VirtualLaser::VirtualLaser(const Laser::ID whichLaser) :
	mCombinedLasers{ whichLaser }
{}

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
void VirtualLaser::configure(RTcontrol &RTcontrol, const int wavelength_nm)
{
	std::future<void> th1{ std::async(&CombinedLasers::setWavelength, &mCombinedLasers, std::ref(RTcontrol), std::ref(wavelength_nm)) };	//Tune the laser wavelength
	std::future<void> th2{ std::async(&VirtualFilterWheel::turnFilterwheels_, &mVirtualFilterWheel, wavelength_nm) };						//Set the filterwheels
	std::future<void> th3{ std::async(&CollectorLens::set, &mCollectorLens, wavelength_nm) };												//Set the collector lens position

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

//Open the Uniblitz shutter
void VirtualLaser::openShutter() const
{
	//Check if the laser internal shutter is open
	mCombinedLasers.isLaserInternalShutterOpen();

	//Open the Uniblitz shutter
	mCombinedLasers.openShutter();
}

//Close the Uniblitz shutter
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
Galvo::Galvo(RTcontrol &RTcontrol, const RTcontrol::RTCHAN whichGalvo, const double posMax, const VirtualLaser *virtualLaser) :
	mRTcontrol{ RTcontrol },
	mWhichGalvo{ whichGalvo },
	mPosMax{ posMax }
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
		mVoltagePerDistance = g_scannerCalib.voltagePerDistance;
		mVoltageOffset = g_scannerCalib.voltageOffset;

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
				mVoltagePerDistance = g_rescannerCalibV750nm.voltagePerDistance;
				mVoltageOffset = g_rescannerCalibV750nm.voltageOffset;
				break;
			case 920:
				mVoltagePerDistance = g_rescannerCalibV920nm.voltagePerDistance;
				mVoltageOffset = g_rescannerCalibV920nm.voltageOffset;
				break;
			case 1040:
				mVoltagePerDistance = g_rescannerCalibV1040nm.voltagePerDistance;
				mVoltageOffset = g_rescannerCalibV1040nm.voltageOffset;
				break;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": The galvo has not been calibrated for the wavelength " + std::to_string(wavelength_nm) + " nm");
			}
			break;
		case Laser::ID::FIDELITY:
			switch (wavelength_nm)
			{
			case 1040:
				mVoltagePerDistance = g_rescannerCalibF1040nm.voltagePerDistance;
				mVoltageOffset = g_rescannerCalibF1040nm.voltageOffset;
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
		positionLinearRamp(mPosMax, -mPosMax, mVoltageOffset + singlebeamVoltageOffset(), OVERRIDE::EN);
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
	}
}

double Galvo::singlebeamVoltageOffset() const
{
	double beamletIndex;
	switch (mRTcontrol.mPMT16Xchan)
	{
	case RTcontrol::PMT16XCHAN::CH00:
		beamletIndex =  7.5;
		break;
	case RTcontrol::PMT16XCHAN::CH01:
		beamletIndex = 6.5;
		break;
	case RTcontrol::PMT16XCHAN::CH02:
		beamletIndex = 5.5;
		break;
	case RTcontrol::PMT16XCHAN::CH03:
		beamletIndex = 4.5;
		break;
	case RTcontrol::PMT16XCHAN::CH04:
		beamletIndex = 3.5;
		break;
	case RTcontrol::PMT16XCHAN::CH05:
		beamletIndex = 2.5;
		break;
	case RTcontrol::PMT16XCHAN::CH06:
		beamletIndex = 1.5;
		break;
	case RTcontrol::PMT16XCHAN::CH07:
		beamletIndex = 0.5;
		break;
	case RTcontrol::PMT16XCHAN::CH08:
		beamletIndex = -0.5;
		break;
	case RTcontrol::PMT16XCHAN::CH09:
		beamletIndex = -1.5;
		break;
	case RTcontrol::PMT16XCHAN::CH10:
		beamletIndex = -2.5;
		break;
	case RTcontrol::PMT16XCHAN::CH11:
		beamletIndex = -3.5;
		break;
	case RTcontrol::PMT16XCHAN::CH12:
		beamletIndex = -4.5;
		break;
	case RTcontrol::PMT16XCHAN::CH13:
		beamletIndex = -5.5;
		break;
	case RTcontrol::PMT16XCHAN::CH14:
		beamletIndex = -6.5;
		break;
	case RTcontrol::PMT16XCHAN::CH15:
		beamletIndex = -7.5;
		break;
	case RTcontrol::PMT16XCHAN::CENTERED:
		beamletIndex = 0.0;//No offset ("centered") for multibeam
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected PMT16X channel unavailable");
	}
	return beamletIndex * mInterBeamletDistance * mVoltagePerDistance;
}

void Galvo::voltageToZero() const
{
	mRTcontrol.pushAnalogSinglet(mWhichGalvo, g_tMinAO, 0 * V);
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
	//Limit the number of steps for long ramps because the buffer of the galvos on the fpga currently only supports 5000 elements
	//For timeStep = 2 us, the max ramp duration is 10 ms. Therefore, 10 ms/ 62.5 us = 160 lines scanned in a single frame
	double timeStep;
	if (mRTcontrol.mHeightPerBeamletPerFrame_pix <= 100)
		timeStep = 2. * us;
	else
		timeStep = 8. * us;

	//The position offset allows to compensate for the axis misalignment of the rescanner wrt the PMT
	mRTcontrol.pushLinearRamp(mWhichGalvo, timeStep, g_lineclockHalfPeriod * mRTcontrol.mHeightPerBeamletPerFrame_pix + mRampDurationFineTuning,
		voltageOffset + mVoltagePerDistance * posInitial, voltageOffset + mVoltagePerDistance * posFinal, override);
}
#pragma endregion "Galvo"

#pragma region "Stages"
Stage::Stage(const double velX, const double velY, const double velZ, const std::vector<LIMIT2> stageSoftPosLimXYZ) :
	mSoftPosLimXYZ{ stageSoftPosLimXYZ }
{
	const std::string stageIDx{ "116049107" };	//X-stage (V-551.4B)
	const std::string stageIDy{ "116049105" };	//Y-stage (V-551.2B)
	const std::string stageIDz{ "0165500631" };	//Z-stage (ES-100)

	//Open the connections to the stage controllers and assign the IDs
	std::cout << "Establishing connection with the stages\n";
	mHandleXYZ.at(XX) = PI_ConnectUSB(stageIDx.c_str());
	mHandleXYZ.at(YY) = PI_ConnectUSB(stageIDy.c_str());
	mHandleXYZ.at(ZZ) = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mHandleXYZ.at(Z) = PI_ConnectUSB(stageIDz.c_str());

	if (mHandleXYZ.at(XX) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");

	if (mHandleXYZ.at(YY) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");

	if (mHandleXYZ.at(ZZ) < 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	std::cout << "Connection with the stages successfully established\n";

	//Download the current position
	mPositionXYZ.XX = downloadPositionSingle_(XX);
	mPositionXYZ.YY = downloadPositionSingle_(YY);
	mPositionXYZ.ZZ = downloadPositionSingle_(ZZ);

	//Download the current velocities
	mVelXYZ.XX = downloadVelSingle_(XX);
	mVelXYZ.YY = downloadVelSingle_(YY);
	mVelXYZ.ZZ = downloadVelSingle_(ZZ);

	configDOtriggers_();				//Configure the stage velocities and DO triggers
	setVelXYZ({ velX, velY, velZ });	//Set the stage velocities
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mHandleXYZ.at(XX));
	PI_CloseConnection(mHandleXYZ.at(YY));
	PI_CloseConnection(mHandleXYZ.at(ZZ));
	//std::cout << "Connection with the stages successfully closed\n";
}

//Recall the current position for the 3 stages
POSITION3 Stage::readPositionXYZ() const
{
	return mPositionXYZ;
}

void Stage::printPositionXYZ() const
{
	std::cout << "Stage X position = " << mPositionXYZ.XX / mm << " mm\n";
	std::cout << "Stage Y position = " << mPositionXYZ.YY / mm << " mm\n";
	std::cout << "Stage Z position = " << mPositionXYZ.ZZ / mm << " mm\n";
}

double Stage::readCurrentPosition_(const Axis axis) const
{
	switch (axis)
	{
	case XX:
		return mPositionXYZ.XX;
	case YY:
		return mPositionXYZ.YY;
	case ZZ:
		return mPositionXYZ.ZZ;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid stage axis");
	}
}

void Stage::setCurrentPosition_(const Axis axis, const double position)
{
	switch(axis)
	{
	case XX:
		mPositionXYZ.XX = position;
	case YY:
		mPositionXYZ.YY = position;
	case ZZ:
		mPositionXYZ.ZZ = position;
	}
}

double Stage::readCurrentVelocity_(const Axis axis) const
{
	switch (axis)
	{
	case XX:
		return mVelXYZ.XX;
	case YY:
		return mVelXYZ.YY;
	case ZZ:
		return mVelXYZ.ZZ;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid stage axis");
	}
}

void Stage::setCurrentVelocity_(const Axis axis, const double velocity)
{
	switch (axis)
	{
	case XX:
		mVelXYZ.XX = velocity;
	case YY:
		mVelXYZ.YY = velocity;
	case ZZ:
		mVelXYZ.ZZ = velocity;
	}
}

//Retrieve the stage position from the controller
double Stage::downloadPositionSingle_(const Axis axis)
{
	double position_mm;	//Position in mm
	if (!PI_qPOS(mHandleXYZ.at(axis), mNstagesPerController, &position_mm))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query position for the stage " + axisToString_(axis));

	return position_mm * mm;	//Multiply by mm to convert from explicit to implicit units
}

//Move the stage to the requested position
void Stage::moveSingle(const Axis axis, const double position)
{
	//Check if the requested position is within range
	if (position < mSoftPosLimXYZ.at(axis).MIN || position > mSoftPosLimXYZ.at(axis).MAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested position is out of the soft limits of the stage " + axisToString_(axis));
	if (position < mTravelRangeXYZ.at(axis).MIN || position > mTravelRangeXYZ.at(axis).MAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The requested position is out of the physical limits of the stage " + axisToString_(axis));

	//Move the stage
	if (readCurrentPosition_(axis) != position) //Move only if the requested position is different from the current position
	{
		const double position_mm{ position / mm };									//Divide by mm to convert from implicit to explicit units
		if (!PI_MOV(mHandleXYZ.at(axis), mNstagesPerController, &position_mm))		//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage " + axisToString_(axis) + " to the target position (maybe hardware limits?)");

		setCurrentPosition_(axis, position);
	}
}

//Move the 2 stages to the requested position
void Stage::moveXY(const POSITION2 positionXY)
{
	moveSingle(XX, positionXY.XX);
	moveSingle(YY, positionXY.YY);
}

//Move the 3 stages to the requested position
void Stage::moveXYZ(const POSITION3 positionXYZ)
{
	moveSingle(XX, positionXYZ.XX);
	moveSingle(YY, positionXYZ.YY);
	moveSingle(ZZ, positionXYZ.ZZ);
}

bool Stage::isMoving(const Axis axis) const
{
	BOOL isMoving;

	if (!PI_IsMoving(mHandleXYZ.at(axis), mNstagesPerController, &isMoving))	//~55 ms to execute this function
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage " + axisToString_(axis));

	return isMoving;
}

void Stage::waitForMotionToStopSingle(const Axis axis) const
{
	std::cout << "Stage " + axisToString_(axis) + " moving to the new position: ";

	BOOL isMoving;
	do {
		if (!PI_IsMoving(mHandleXYZ.at(axis), mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + axisToString_(axis));

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
		if (!PI_IsMoving(mHandleXYZ.at(XX), mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mHandleXYZ.at(YY), mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mHandleXYZ.at(ZZ), mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << ".";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}

void Stage::stopAll() const
{
	PI_StopAll(mHandleXYZ.at(XX));
	PI_StopAll(mHandleXYZ.at(YY));
	PI_StopAll(mHandleXYZ.at(ZZ));

	std::cout << "Stages stopped\n";
}

//Request the velocity of the stage
double Stage::downloadVelSingle_(const Axis axis) const
{
	double vel_mmps;
	if (!PI_qVEL(mHandleXYZ.at(axis), mNstagesPerController, &vel_mmps))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the velocity for the stage " + axisToString_(axis));

	//std::cout << vel_mmps << " mm/s\n";
	return vel_mmps * mmps;					//Multiply by mmps to convert from explicit to implicit units
}

//Set the velocity of the stage
void Stage::setVelSingle(const Axis axis, const double vel)
{
	//Check if the requested vel is valid
	if (vel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The velocity must be >0 for the stage " + axisToString_(axis));

	//Update the vel if different
	if (readCurrentVelocity_(axis) != vel)
	{
		const double vel_mmps{ vel / mmps };		//Divide by mmps to convert implicit to explicit units
		if (!PI_VEL(mHandleXYZ.at(axis), mNstagesPerController, &vel_mmps))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the velocity for the stage " + axisToString_(axis));

		setCurrentVelocity_(axis, vel);
		//std::cout << "stage vel updated\n"; //For debugging
	}
}

//Set the velocity of the stage 
void Stage::setVelXYZ(const VELOCITY3 velXYZ)
{
	setVelSingle(XX, velXYZ.XX);
	setVelSingle(YY, velXYZ.YY);
	setVelSingle(ZZ, velXYZ.ZZ);
}

void Stage::printVelXYZ() const
{
	std::cout << "Stage X vel = " << mVelXYZ.XX / mmps << " mm/s\n";
	std::cout << "Stage Y vel = " << mVelXYZ.YY / mmps << " mm/s\n";
	std::cout << "Stage Z vel = " << mVelXYZ.ZZ / mmps << " mm/s\n";
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
	if (!PI_qCTO(mHandleXYZ.at(axis), &DOchan_int, &triggerParamID_int, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger config for the stage " + axisToString_(axis));

	//std::cout << value << "\n";
	return value;
}

void Stage::setDOtriggerParamSingle(const Axis axis, const DIOCHAN DOchan, const DOPARAM triggerParamID, const double value) const
{
	const int triggerParamID_int{ static_cast<int>(triggerParamID) };
	const int DOchan_int{ static_cast<int>(DOchan) };
	if (!PI_CTO(mHandleXYZ.at(axis), &DOchan_int, &triggerParamID_int, &value, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger config for the stage " + axisToString_(axis));
}

void Stage::setDOtriggerParamAll(const Axis axis, const DIOCHAN DOchan, const double triggerStep, const DOTRIGMODE triggerMode, const double startThreshold, const double stopThreshold) const
{
	if (triggerStep <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The trigger step must be >0");

	if (startThreshold < mTravelRangeXYZ.at(axis).MIN || startThreshold > mTravelRangeXYZ.at(axis).MAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The start position is out of the physical limits of the stage " + axisToString_(axis));

	if (stopThreshold < mTravelRangeXYZ.at(axis).MIN || stopThreshold > mTravelRangeXYZ.at(axis).MAX)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stop position is out of the physical limits of the stage " + axisToString_(axis));


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
	if (!PI_qTRO(mHandleXYZ.at(axis), &DOchan_int, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query the trigger EN/DIS stage for the stage " + axisToString_(axis));

	//std::cout << triggerState << "\n";
	return triggerState;
}

//Enable or disable the stage DO
void Stage::setDOtriggerEnabled(const Axis axis, const DIOCHAN DOchan, const BOOL triggerState) const
{
	const int DOchan_int{ static_cast<int>(DOchan) };
	if (!PI_TRO(mHandleXYZ.at(axis), &DOchan_int, &triggerState, 1))
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to set the trigger EN/DIS state for the stage " + axisToString_(axis));
}

//Each stage driver has 4 DO channels that can be used to monitor the stage position, motion, etc
//Print out the relevant parameters
void Stage::printStageConfig(const Axis axis, const DIOCHAN chan) const
{
	switch (axis)
	{
	case XX:
		//Only DO1 is wired to the FPGA
		if (chan != DIOCHAN::D1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString_(axis));
		break;
	case YY:
		//Only DO1 is wired to the FPGA
		if (chan != DIOCHAN::D1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 is currently wired to the FPGA for the stage " + axisToString_(axis));
		break;
	case ZZ:
		//Only DO1 and DO2 are wired to the FPGA
		if (chan != DIOCHAN::D1 || chan != DIOCHAN::D2)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Only DO1 and DO2 are currently wired to the FPGA for the stage " + axisToString_(axis));
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

	std::cout << "Configuration for the stage = " << axisToString_(axis) << ", DOchan = " << static_cast<int>(chan) << ":\n";
	std::cout << "is DO trigger enabled? = " << triggerState << "\n";
	std::cout << "Trigger step = " << triggerStep_mm << " mm\n";
	std::cout << "Trigger mode = " << triggerMode << "\n";
	std::cout << "POLARITY = " << polarity << "\n";
	std::cout << "Start threshold position = " << startThreshold_mm << " mm\n";
	std::cout << "Stop threshold position = " << stopThreshold_mm << " mm\n";
	std::cout << "Trigger position = " << triggerPosition_mm << " mm\n";
	std::cout << "Vel = " << vel / mmps << " mm/s\n\n";
}

//DO1 and DO2 of z-stage are used to trigger the stack acquisition. Currently only DO2 is used as trigger. See the implementation on LV
void Stage::configDOtriggers_() const
{

	//STAGE X. DO1 (PIN5 of the RS232 connector). DI2 (PIN2 of the RS232 connector). AFAIR, DI1 was not working properly as an an input pin (disabled by the firmware?)
	const DIOCHAN XDO1{ DIOCHAN::D1 };
	setDOtriggerEnabled(XX, XDO1, true);																//Enable DO1 output
	setDOtriggerParamSingle(XX, XDO1, DOPARAM::TRIGMODE, static_cast<double>(DOTRIGMODE::INMOTION));	//Configure DO1 as motion monitor


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
	setDOtriggerEnabled(ZZ, ZDO2, true);																//Enable DO2 output
	setDOtriggerParamSingle(ZZ, ZDO2, DOPARAM::TRIGMODE, static_cast<double>(DOTRIGMODE::INMOTION));	//Configure DO2 as motion monitor
}

std::string Stage::axisToString_(const Axis axis) const
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
#pragma endregion "Stages"

#pragma region "Vibratome"
Vibratome::Vibratome(const FPGA &fpga, Stage &stage) :
	mFpga{ fpga },
	mStage{ stage }
{}

//Start or stop running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::pushStartStopButton() const
{
	const int pulsewidth{ 100 * ms }; //in ms. It has to be longer than~ 12 ms, otherwise the vibratome is not triggered

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), NiFpga_FPGAvi_ControlBool_VTstart, true));

	Sleep(static_cast<DWORD>(pulsewidth / ms));

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
}

void Vibratome::slice(const double planeToCutZ)
{
	mStage.setVelXYZ(mStageConveyingVelXYZ);													//Change the velocity to move the sample to the vibratome
	mStage.moveXYZ({ mStageInitialSlicePosXY.XX, mStageInitialSlicePosXY.YY, planeToCutZ });	//Position the sample in front of the vibratome's blade
	mStage.waitForMotionToStopAll();

	mStage.setVelSingle(Stage::YY, mSlicingVel);							//Change the y vel for slicing
	pushStartStopButton();													//Turn on the vibratome
	mStage.moveSingle(Stage::YY, mStageFinalSlicePosY);						//Slice the sample: move the stage y towards the blade
	mStage.waitForMotionToStopSingle(Stage::YY);							//Wait until the motion ends
	mStage.setVelSingle(Stage::YY, mStageConveyingVelXYZ.YY);				//Set back the y vel to move the sample back to the microscope

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

	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), selectedChannel, true));

	if (duration >= minDuration)
		Sleep(static_cast<DWORD>((duration - delay)/ms));
	else
	{
		Sleep(static_cast<DWORD>((minDuration - delay)/ms));
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << 1. * minDuration / ms << "ms" << "\n";
	}
	FPGAfunc::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.handle(), selectedChannel, false));
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