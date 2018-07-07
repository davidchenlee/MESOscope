#include "Devices.h"

#pragma region "Vibratome"
Vibratome::Vibratome(const FPGAapi::Session &fpga): mFpga(fpga){}

Vibratome::~Vibratome() {}

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::startStop()
{
	const int SleepTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 1));

	Sleep(SleepTime);

	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 0));
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
void Vibratome::sendCommand(const double pulseDuration, const VibratomeChannel channel) const
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const int minPulseDuration = 10;	//in ms
	const int delay = 1;				//Used to roughly calibrate the pulse length
	const int dt_ms = (int)pulseDuration / ms;

	switch (channel)
	{
	case VibratomeBack:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_back;
		break;
	case VibratomeForward:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_forward;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected vibratome channel unavailable");
	}

	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 1));

	if (dt_ms >= minPulseDuration)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minPulseDuration - delay);
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << minPulseDuration << "ms" << std::endl;
	}
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 0));
}
#pragma endregion "Vibratome"

#pragma region "Resonant scanner"
ResonantScanner::ResonantScanner(const FPGAapi::Session &fpga): mFpga(fpga){};
ResonantScanner::~ResonantScanner() {};

//Start or stop the resonant scanner
void ResonantScanner::run(const bool state){
	
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool)state));
}

//Set the control voltage that determines the scanning amplitude
void ResonantScanner::setControl_V(const double Vcontrol_V)
{
	if (Vcontrol_V > mVMAX_V) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_V) + " V" );

	mVcontrol_V = Vcontrol_V;
	mFFOV_um = Vcontrol_V / mVoltPerUm;

	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, FPGAapi::convertVoltToI16(mVcontrol_V)));
}

//Set the FFOV
void ResonantScanner::setFFOV_um(const double FFOV_um)
{
	mVcontrol_V = FFOV_um * mVoltPerUm;
	mFFOV_um = FFOV_um;

	if (mVcontrol_V > mVMAX_V) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_V) + " V");

	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, FPGAapi::convertVoltToI16(mVcontrol_V)));
}

void ResonantScanner::turnOn_um(const double FFOV_um)
{
	setFFOV_um(FFOV_um);
	Sleep(mDelayTime_ms);
	run(1);
}

void ResonantScanner::turnOn_V(const double Vcontrol_V)
{
	setControl_V(Vcontrol_V);
	Sleep(mDelayTime_ms);
	run(1);
}

void ResonantScanner::turnOff()
{
	run(0);
	Sleep(mDelayTime_ms);
	setControl_V(0);
}


double ResonantScanner::convertUm2Volt(double amplitude_um) const
{
	return amplitude_um * mVoltPerUm;
}

#pragma endregion "Resonant scanner"

#pragma region "Shutters"

Shutter::Shutter(const FPGAapi::Session &fpga, ShutterID ID) : mFpga(fpga)
{
	switch (ID)
	{
	case Shutter1:
		mID = NiFpga_FPGAvi_ControlBool_Shutter1;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected shutter unavailable");
	}

}

Shutter::~Shutter() {}

void Shutter::open() const
{
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 1));
}

void Shutter::close() const
{
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 0));
}

void Shutter::pulseHigh() const
{
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 1));

	Sleep(mDelayTime_ms);

	FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 0));
}
#pragma endregion "Shutters"

#pragma region "Image"
Image::Image(const FPGAapi::Session &fpga) : mFpga(fpga)
{
	mBufArray_A = new U32[nPixAllFrames]();

	mNelemBufArray_B = new int[nBufArrays]();
	mBufArray_B = new U32*[nBufArrays];
	for (int i = 0; i < nBufArrays; i++)
		mBufArray_B[i] = new U32[nPixAllFrames]();

	image = new unsigned char[nPixAllFrames]();
};

Image::~Image()
{
	//Stop the FIFOpc. Before implementing this function, the computer used to crash if the code was run after terminated under exception.
	//(I think) this is because the access to the FIFO used to remain open and clashed with the next FIFO call
	stopFIFOpc(); 

	delete[] mBufArray_A;
	delete[] mNelemBufArray_B;

	//clean up the buffer arrays
	for (int i = 0; i < nBufArrays; ++i) {
		delete[] mBufArray_B[i];
	}
	delete[] mBufArray_B;
	delete[] image;
	//std::cout << "Image destructor called\n";
};

//Establish a connection between FIFOpc and FIFOfpga
void Image::startFIFOpc() const
{
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}

//Configure FIFOpc
void Image::configureFIFOpc(const U32 depth) const
{
	U32 actualDepth;
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << std::endl;
}

//Stop the connection between FIFOpc and FIFOfpga
void Image::stopFIFOpc() const
{
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
	//std::cout << "stopFIFO called\n";
}

//Remaining elements in FIFOpc
void Image::readRemainingFIFOpc() const
{
	U32 rem;
	U32 *dummy = new U32[0];
	FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, dummy, 0, mTimeout_ms, &rem));

	std::cout << "remaining = " << rem << std::endl;
}


//Read the data in FIFOpc
void Image::readFIFOpc()
{
	//TODO: save the data concurrently
	//I ran a test and found that two 32-bit FIFOfpga have a larger bandwidth than a single 64 - bit FIFOfpga
	//Test if the bandwidth can be increased by using 'NiFpga_AcquireFifoReadElementsU32'.Ref: http://zone.ni.com/reference/en-XX/help/372928G-01/capi/functions_fifo_read_acquire/
	//pass an array to a function: https://stackoverflow.com/questions/2838038/c-programming-malloc-inside-another-function
	//review of pointers and references in C++: https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html

	//Declare and start a stopwatch [2]
	double duration;
	auto t_start = std::chrono::high_resolution_clock::now();

	U32 *dummy = new U32[0];

	while (mNelemReadFIFO_A < nPixAllFrames || mNelemReadFIFO_B < nPixAllFrames)
	{
		Sleep(mReadFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time to max transfer bandwidth

		//FIFOpc A
		if (mNelemReadFIFO_A < nPixAllFrames)
		{
			FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dummy, 0, mTimeout_ms, &mNremainFIFO_A));
			//std::cout << "Number of elements remaining in FIFOpc A: " << mNremainFIFO_A << std::endl;

			if (mNremainFIFO_A > 0)
			{
				mNelemReadFIFO_A += mNremainFIFO_A;
				FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, mBufArray_A, mNremainFIFO_A, mTimeout_ms, &mNremainFIFO_A));
			}
		}

		//FIFOpc B
		if (mNelemReadFIFO_B < nPixAllFrames)		//Skip if all the data have already been transferred (i.e. nElemReadFIFO_A = nPixAllFrames)
		{
			//By requesting 0 elements from FIFOpc, the function returns the number of elements available. If no data available so far, then nRemainFIFO_B = 0 is returned
			FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, dummy, 0, mTimeout_ms, &mNremainFIFO_B));
			//std::cout << "Number of elements remaining in FIFOpc B: " << mNremainFIFO_B << std::endl;

			//If there are data available in the FIFOpc, retrieve it
			if (mNremainFIFO_B > 0)
			{
				mNelemReadFIFO_B += mNremainFIFO_B;							//Keep track of the number of elements read so far
				mNelemBufArray_B[mCounterBufArray_B] = mNremainFIFO_B;		//Keep track of how many elements are in each buffer array												

				//Read the elements in the FIFOpc
				FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, mBufArray_B[mCounterBufArray_B], mNremainFIFO_B, mTimeout_ms, &mNremainFIFO_B));

				if (mCounterBufArray_B >= nBufArrays)
					throw ImageException((std::string)__FUNCTION__ + ": Buffer array overflow");

				mCounterBufArray_B++;
			}
		}

		mTimeoutCounter_iter--;

		//Transfer timeout
		if (mTimeoutCounter_iter == 0)
			throw ImageException((std::string)__FUNCTION__ + ": FIFOpc downloading timeout");
	}

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

	std::cout << "FIFO bandwidth: " << 2 * 32 * nPixAllFrames / duration / 1000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer arrays used: " << (U32)mCounterBufArray_B << std::endl; //Print out how many buffer arrays were actually used
	std::cout << "Total of elements read: " << mNelemReadFIFO_A << "\t" << mNelemReadFIFO_B << std::endl; //Print out the total number of elements read

	//If all the expected data is NOT read successfully
	if (mNelemReadFIFO_A != nPixAllFrames || mNelemReadFIFO_B != nPixAllFrames)
		throw ImageException((std::string)__FUNCTION__ + ": More or less FIFO elements received than expected ");

	delete[] dummy;
}

//Returns a single 1D array with the chucks of data stored in the buffer arrays
void Image::unpackBuffer()
{
	//ReadFifo gives chuncks of data. Store each chunck in a separate buffer array. I think I CANNOT just make a long, concatenated 1D array because I have to pass individual arrays to the ReadFifo
	//When multiplexing later on, each U32 element in bufArray_B must be split in 8 parts of 4-bits each

	const bool debug = 0;
	double upscaledCount;

	U32 pixIndex = 0;	//Index for the image pixel
	for (int ii = 0; ii < mCounterBufArray_B; ii++)
	{
		for (int jj = 0; jj < mNelemBufArray_B[ii]; jj++)
		{
			upscaledCount = std::floor(upscaleU8 * mBufArray_B[ii][jj]); //Upscale the photoncount to a 8-bit number

			if (upscaledCount > _UI8_MAX)
				upscaledCount = _UI8_MAX;
				//throw ImageException((std::string)__FUNCTION__ + ": Upscaled photoncount overflow");

			image[pixIndex] = (unsigned char)upscaledCount;

			//For debugging. Generate numbers from 1 to nPixAllFrames with +1 increaments
			if (debug)
				image[pixIndex] = pixIndex + 1;

			pixIndex++;
		}
	}
}

//The microscope scans bidirectionally. The pixel order is reversed every other line.
//Later on, write the tiff directly from the buffer arrays. To deal with segmented pointers, use memcpy, memset, memmove or the Tiff versions for such functions
//memset http://www.cplusplus.com/reference/cstring/memset/
//memmove http://www.cplusplus.com/reference/cstring/memmove/
//One idea is to read bufArray_B line by line (1 line = Width_pix x 1) and save it to file using TIFFWriteScanline
void Image::correctInterleavedImage()
{
	unsigned char *auxLine = new unsigned char[widthPerFrame_pix]; //one line to store the temp data. In principle I could just use half the size, but why bothering...

	//Reverse the pixel order every other line
	for (int lineIndex = 1; lineIndex < heightPerFrame_pix; lineIndex += 2)
	{
		//save the data in an aux array
		for (int pixIndex = 0; pixIndex < widthPerFrame_pix; pixIndex++)
			auxLine[pixIndex] = image[lineIndex*widthPerFrame_pix + (widthPerFrame_pix - pixIndex - 1)];	//TODO: use memcpy
																											//write the data back in reversed order
		for (int pixIndex = 0; pixIndex < widthPerFrame_pix; pixIndex++)
			image[lineIndex*widthPerFrame_pix + pixIndex] = auxLine[pixIndex];		//TODO: use memcpy
	}
	delete[] auxLine;
}


void Image::acquire(const std::string filename)
{
	if (enableFIFOfpga)
		mFpga.enableFIFOfpga();	//Enable pushing data to FIFOfpga

	startFIFOpc();		//Establish the connection between FIFOfpga and FIFOpc
	mFpga.triggerRT();	//Trigger the RT sequence. If triggered too early, FIFOfpga will probably overflow

	if (enableFIFOfpga)
	{
		try
		{
			readFIFOpc();		//Read the data in FIFOpc
			unpackBuffer();		//Move the chuncks of data to a buffer array
			correctInterleavedImage();
			saveAsTiff(filename);
		}
		catch (const ImageException &e) //Notify the exception and move to the next iteration
		{
			std::cout << "An ImageException has occurred in: " << e.what() << std::endl;
		}
	}

}

void Image::saveAsTiff(std::string filename)
{
	if (!overrideSaving)
		filename = file_exists(filename);

	TIFF *tiffHandle = TIFFOpen((foldername + filename + ".tif").c_str(), "w");

	if (tiffHandle == nullptr)
		throw ImageException((std::string)__FUNCTION__ + ": Saving Tiff failed");

	//TAGS
	TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, widthPerFrame_pix);					//Set the width of the image
	TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, heightPerFrame_pix);					//Set the height of the image
	TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);								//Set number of channels per pixel
	TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);									//Set the size of the channels
	TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);					//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
	//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);				//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
	TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);				//Single channel with min as black

	tsize_t bytesPerLine = widthPerFrame_pix;			//Length in memory of one row of pixel in the image.
	unsigned char *buffer = nullptr;					//Buffer used to store the row of pixel information for writing to file

														//Allocating memory to store pixels of current row
	if (TIFFScanlineSize(tiffHandle))
		buffer = (unsigned char *)_TIFFmalloc(bytesPerLine);
	else
		buffer = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(tiffHandle));

	//Set the strip size of the file to be size of one row of pixels
	TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, widthPerFrame_pix));

	//Now writing image to the file one strip at a time. CURRENTLY ONLY ONE FRAME IS SAVED!!!
	for (int row = 0; row < heightPerFrame_pix; row++)
	{
		memcpy(buffer, &image[(heightPerFrame_pix - row - 1)*bytesPerLine], bytesPerLine);    // check the index here, and figure tiffHandle why not using h*bytesPerLine
		if (TIFFWriteScanline(tiffHandle, buffer, row, 0) < 0)
			break;
	}

	//Close the output file
	(void)TIFFClose(tiffHandle);

	//Destroy the buffer
	if (buffer)
		_TIFFfree(buffer);
}

void Image::saveAsTxt(const std::string filename)
{
	std::ofstream fileHandle;								//Create output file
	fileHandle.open(foldername + filename + ".txt");		//Open the file

	for (int ii = 0; ii < nPixAllFrames; ii++)
		fileHandle << (int)image[ii] << std::endl;		//Write each element

	fileHandle.close();									//Close the txt file
}
#pragma endregion "Image"


#pragma region "Pockels cells"
//Curently the output is hard coded on the FPGA side and triggered by the 'frame gate'
PockelsCell::PockelsCell(FPGAapi::RTsequence &sequence, const RTchannel pockelsChannel, const int wavelength_nm) : mSequence(sequence), mPockelsChannel(pockelsChannel), mWavelength_nm(wavelength_nm)
{
	if (mPockelsChannel != POCKELS1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell channel unavailable");

	switch (mPockelsChannel)
	{
	case POCKELS1:
		mScalingChannel = SCALING1;
		break;
	}

	//Initialize all the scaling factors to 1.0. In LV, I could not sucessfully default the LUT as 0d16384 = 0b0100000000000000 = 1 for a fixed point Fx2.14
	for (int ii = 0; ii < nFrames; ii++)
		mSequence.pushAnalogSingletFx2p14(mScalingChannel, 1.0);
}

//Do not set the output to 0 with the destructor to allow holding on the last value
PockelsCell::~PockelsCell() {}


void PockelsCell::pushSinglet(const double timeStep, const double AO_V) const
{
	if (AO_V < 0) throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell output voltage must be positive");

	mSequence.pushAnalogSinglet(mPockelsChannel, timeStep, AO_V);
}

void PockelsCell::pushPowerSinglet(const double timeStep, const double P_mW) const
{
	if (P_mW < 0) throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell output power must be positive");

	mSequence.pushAnalogSinglet(mPockelsChannel, timeStep, convertPowerToVoltage_V(P_mW));
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void PockelsCell::voltageLinearRamp(const double timeStep, const double rampDuration, const double Vi_V, const double Vf_V) const
{
	if (Vi_V < 0 || Vf_V < 0) throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell output voltage must be positive");

	mSequence.pushLinearRamp(mPockelsChannel, timeStep, rampDuration, Vi_V, Vf_V);
}

//Ramp the pockels cell modulation during a frame acquisition. The bandwidth is limited by the HV amp = 40 kHz ~ 25 us
void  PockelsCell::powerLinearRamp(const double timeStep, const double rampDuration, const double Pi_mW, const double Pf_mW) const
{
	if (Pi_mW < 0 || Pf_mW < 0) throw std::invalid_argument((std::string)__FUNCTION__ + ": Pockels cell output voltage must be positive");

	mSequence.pushLinearRamp(mPockelsChannel, timeStep, rampDuration, convertPowerToVoltage_V(Pi_mW), convertPowerToVoltage_V(Pf_mW));
}

void PockelsCell::voltageToZero() const
{
	mSequence.pushAnalogSinglet(mPockelsChannel, AO_tMIN_us, 0 * V);
}

//Scale the pockels modulation across all the frames following a linear ramp
void PockelsCell::scalingLinearRamp(const double Si, const double Sf)
{
	if (Si < 0 || Sf < 0 || Si > 4 || Sf > 4) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested scaling factor is outside the range 0-4");
	if (nFrames < 2) throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be > 1");

	mSequence.clearQueue(mScalingChannel);	//Delete the default scaling factors of 1.0s created in the PockelsCell constructor

	for (int ii = 0; ii < nFrames; ii++)
		mSequence.pushAnalogSingletFx2p14(mScalingChannel, Si + (Sf - Si) / (nFrames-1) * ii);

}


double PockelsCell::convertPowerToVoltage_V(const double power_mW) const
{
	double a, b, c;		//Calibration parameters

	if (mWavelength_nm == 750) {
		a = 433.6;
		b = 0.647;
		c = 0.042;
	}
	else if (mWavelength_nm == 940) {
		a = 245.4;
		b = 0.513;
		c = -0.050;
	}
	else if (mWavelength_nm == 1040) {
		a = 100.0;
		b = 0.458;
		c = 0.055;
	}
	else
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Laser wavelength " + std::to_string(mWavelength_nm) + " nm currently not calibrated");

	double arg = sqrt(power_mW / a);
	if (arg > 1) throw std::invalid_argument((std::string)__FUNCTION__ + ": Arg of asin is greater than 1");

	return asin(arg)/b + c;
}
#pragma endregion "Pockels cells"


#pragma region "Galvo"

Galvo::Galvo(FPGAapi::RTsequence &sequence, const RTchannel galvoChannel): mSequence(sequence), mGalvoChannel(galvoChannel)
{
	if ( mGalvoChannel != GALVO1 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected galvo channel unavailable");
}

Galvo::~Galvo() {}

double Galvo::convertPositionToVoltage(const double position_um) const
{
	return position_um * voltPerUm;
}

void Galvo::pushSinglet(const double timeStep, const double AO_V) const
{
	mSequence.pushAnalogSinglet(mGalvoChannel, timeStep, AO_V);
}

void Galvo::voltageLinearRamp(const double timeStep, const double rampLength, const double Vi_V, const double Vf_V) const
{
	mSequence.pushLinearRamp(mGalvoChannel, timeStep, rampLength, Vi_V, Vf_V);
}

void Galvo::positionLinearRamp(const double timeStep, const double rampLength, const double xi_V, const double xf_V) const
{
	mSequence.pushLinearRamp(mGalvoChannel, timeStep, rampLength, convertPositionToVoltage(xi_V), convertPositionToVoltage(xf_V));
}

void Galvo::voltageToZero() const
{
	mSequence.pushAnalogSinglet(mGalvoChannel, AO_tMIN_us, 0 * V);
}

#pragma endregion "Galvo"


#pragma region "mPMT"

mPMT::mPMT()
{
	mSerial = new serial::Serial(port, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
}

mPMT::~mPMT()
{

}

std::vector<uint8_t> mPMT::sendCommand(std::vector<uint8_t> command_array)
{
	command_array.push_back(sumCheck(command_array, command_array.size()));	//Append the sumcheck

	std::string TxBuffer(command_array.begin(), command_array.end()); //Convert the vector<char> to string
	TxBuffer += "\r";	//End the command line with CR
	//printHex(TxBuffer); //For debugging

	std::vector<uint8_t> RxBuffer;
	mSerial->write("\r");						//Wake up the mPMT
	mSerial->read(RxBuffer, RxBufferSize);		//Read the state: 0x0D(0d13) for ready, or 0x45(0d69) for error

	//Throw an error if RxBuffer is empty or CR is NOT returned
	if ( RxBuffer.empty() || RxBuffer.at(0) != 0x0D )
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure waking up the mPMT microcontroller");
	
	//printHex(RxBuffer); //For debugging

	RxBuffer.clear(); //Flush the buffer
	mSerial->write(TxBuffer);
	mSerial->read(RxBuffer, RxBufferSize);
	
	//Throw an error if RxBuffer is empty
	if (RxBuffer.empty())
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failure reading the mPMT microcontroller");

	//printHex(RxBuffer); //For debugging

	return RxBuffer;
}

//Return the sumcheck of all the elements in the array
uint8_t mPMT::sumCheck(const std::vector<uint8_t> charArray, const int nElements)
{
	uint8_t sum = 0;
	for (int ii = 0; ii < nElements; ii++)
		sum += charArray.at(ii);

	return sum;
}

void mPMT::readAllGain()
{
	std::vector<uint8_t> parameters = sendCommand({'I'});

	//Check that the chars returned by the mPMT are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'I' || parameters.at(17) != sumCheck(parameters, parameters.size() - 2))
		std::cout << "Warning: CheckSum mismatch in " + (std::string)__FUNCTION__ << std::endl;
	
	//Print out the gains
	std::cout << "mPMT gains:" << std::endl;
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << (int)parameters.at(ii) << std::endl;		
}

void mPMT::setSingleGain(const int channel, const int gain)
{
	//Check that the input parameters are within range
	if (channel < 1 || channel > 16)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": mPMT channel number out of range (1-16)");

	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": mPMT gain out of range (0-255)");


	std::vector<uint8_t> parameters = sendCommand({'g', (uint8_t)channel, (uint8_t)gain});
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the mPMT are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'g' && parameters.at(1) == (uint8_t)channel && parameters.at(2) == (uint8_t)gain && parameters.at(3) == sumCheck(parameters, parameters.size()-2))
		std::cout << "mPMT channel " << channel << " successfully set to " << gain << std::endl;
	else
		std::cout << "Warning: CheckSum mismatch in " + (std::string)__FUNCTION__ << std::endl;
}

void mPMT::setAllGainToZero()
{
	std::vector<uint8_t> parameters = sendCommand({ 'R' }); //The manual says that this sets all the gains to 255, but it really does it to 0
															//printHex(parameters);	//For debugging

	//Check that the chars returned by the mPMT are correct. The second char returned is the sumcheck
	if (parameters.at(0) == 'R' && parameters.at(1) == 'R')
		std::cout << "All mPMT gains successfully set to 0" << std::endl;
}

void mPMT::setAllGain(const int gain)
{
	if (gain < 0 || gain > 255)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": mPMT gain must be in the range 0-255");

	std::vector<uint8_t> parameters = sendCommand({ 'S', (uint8_t)gain });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the mPMT are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) == 'S' && parameters.at(1) == (uint8_t)gain && parameters.at(2) == sumCheck(parameters, parameters.size() - 2))
		std::cout << "All mPMT gains successfully set to " << gain << std::endl;
	else
		std::cout << "Warning: CheckSum mismatch in " + (std::string)__FUNCTION__ << std::endl;
}

void mPMT::setAllGain(std::vector<uint8_t> gains)
{
	//Check that the input parameters are within range
	if (gains.size() != 16)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Gain array must have 16 elements");

	for (int ii = 0; ii < 16; ii++)
		if (gains.at(ii) < 0 || gains.at(ii) > 255)
			throw std::invalid_argument((std::string)__FUNCTION__ + ":  mPMT gain #" + std::to_string(ii) + " out of range (0-255)");

	gains.insert(gains.begin(), 'G');	//Prepend the command
	std::vector<uint8_t> parameters = sendCommand({ gains });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the mPMT are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'G' || parameters.at(17) != sumCheck(parameters, parameters.size() - 2))
		std::cout << "Warning: CheckSum mismatch in " + (std::string)__FUNCTION__ << std::endl;

	//Print out the gains
	std::cout << "mPMT gains successfully set to:" << std::endl;
	for (int ii = 1; ii <= 16; ii++)
		std::cout << "Gain #" << ii << " (0-255) = " << (int)parameters.at(ii) << std::endl;

}

void mPMT::readTemp()
{
	std::vector<uint8_t> parameters = sendCommand({ 'T' });
	//printHex(parameters);	//For debugging

	//Check that the chars returned by the mPMT are correct. Sum-check the chars till the last two, which are the returned sumcheck and CR
	if (parameters.at(0) != 'T' || parameters.at(4) != sumCheck(parameters, parameters.size() - 2))
		std::cout << "Warning: CheckSum mismatch in " + (std::string)__FUNCTION__ << std::endl;

	const int TEMPH = (int)(parameters.at(1));
	const int TEMPL = (int)(parameters.at(2));
	const double temp_C = TEMPH + 0.01 * TEMPL; //According to the manual

	const int alertTemp_C = (int)(parameters.at(3));

	std::cout << "mPMT temperature = " << temp_C << " \370C" << std::endl;
	std::cout << "mPMT alert temperature = " << alertTemp_C <<  " \370C" << std::endl;
}


#pragma endregion "mPMT"



#pragma region "Filterwheel"
Filterwheel::Filterwheel(const FilterwheelID ID): mID(ID)
{
	switch (ID)
	{
	case FW1:
		port = "COM6";
		break;
	case FW2:
		port = "bbb";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected filterwheel unavailable");
	}

	mSerial = new serial::Serial(port, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
	this->readFilterPosition_();	//Read the current filter position
}

Filterwheel::~Filterwheel()
{
	mSerial->close();
}

void Filterwheel::readFilterPosition_()
{
	const std::string TxBuffer = "pos?\r";	//Command to the filterwheel
	std::string RxBuffer;					//Reply from the filterwheel
	const int RxBufSize = 10;

	mSerial->write(TxBuffer);
	mSerial->read(RxBuffer, RxBufSize);

	//Delete echoed command. Echoing could be disabled on the laser but deleting it is more general and safer
	std::string::size_type ii = RxBuffer.find(TxBuffer);
	if (ii != std::string::npos)
		RxBuffer.erase(ii, TxBuffer.length());

	//Delete CR and >
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
	//RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

	mPosition = static_cast<FilterColor>(std::atoi(RxBuffer.c_str()));	//convert string to int, then to FilterColor
	//std::cout << RxBuffer << std::endl;
}

FilterColor Filterwheel::readFilterPosition() const
{
	return mPosition;
}

void Filterwheel::setFilterPosition(const FilterColor color)
{
	if (color != mPosition)
	{
		std::string TxBuffer = "pos=" + std::to_string(color) + "\r";
		mSerial->write(TxBuffer);
		mPosition = color;
	}
}
#pragma endregion "Filterwheel"

#pragma region "Lasers"
Laser::Laser()
{
	mSerial = new serial::Serial(port, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
	this->downloadWavelength();
};

Laser::~Laser() {};

void Laser::downloadWavelength()
{
	const std::string TxBuffer = "?VW";		//Command to the filterwheel
	std::string RxBuffer;					//Reply from the filterwheel
	const int RxBufSize = 20;

	mSerial->write(TxBuffer + "\r");
	mSerial->read(RxBuffer, RxBufSize);

	//Delete echoed command. Echoing could be disabled on the laser but deleting it is more general and safer
	std::string keyword = "?VW ";
	std::string::size_type i = RxBuffer.find(keyword);
	if (i != std::string::npos)
		RxBuffer.erase(i, keyword.length());

	//Delete "CHAMELEON>". This frase could be disabled on the laser, but deleting it is more general and safer
	keyword = "CHAMELEON>";
	i = RxBuffer.find(keyword);
	if (i != std::string::npos)
		RxBuffer.erase(i, keyword.length());

	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

	mWavelength = static_cast<FilterColor>(std::atoi(RxBuffer.c_str()));	//convert string to int
	std::cout << RxBuffer << std::endl;
}

int Laser::readWavelength_nm() const
{
	return mWavelength;
}

void Laser::setWavelength()
{
	const std::string TxBuffer = "VW=800";		//Command to the filterwheel
	std::string RxBuffer;						//Reply from the filterwheel
	const int RxBufSize = 256;

	mSerial->write(TxBuffer + "\r");
	mSerial->read(RxBuffer, RxBufSize);

	this->downloadWavelength();
}

#pragma endregion "Lasers"

#pragma region "Stages"
Stage::Stage()
{
	//Open the connections to the stage controllers and assign IDs
	mID[xx] = PI_ConnectUSB(mStageName_x.c_str());
	mID[yy] = PI_ConnectUSB(mStageName_y.c_str());
	mID[zz] = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 4 for "COM4" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID[zz] = PI_ConnectUSB(mStageName_z.c_str());

	if (mID[xx] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");
	if (mID[yy] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");
	if (mID[zz] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	//Record the current position
	mPosition3_mm[xx] = downloadPosition_mm(xx);
	mPosition3_mm[yy] = downloadPosition_mm(yy);
	mPosition3_mm[zz] = downloadPosition_mm(zz);
}

Stage::~Stage()
{
	//Close the Connections
	PI_CloseConnection(mID[xx]);
	PI_CloseConnection(mID[yy]);
	PI_CloseConnection(mID[zz]);
	std::cout << "Stages connection closed.\n";
}

//Recall the current position for the 3 stages
double3 Stage::readPosition3_mm() const
{
	return mPosition3_mm;
}

void Stage::printPosition3() const
{
	std::cout << "Stage X position = " << mPosition3_mm[xx] << " mm" << std::endl;
	std::cout << "Stage Y position = " << mPosition3_mm[yy] << " mm" << std::endl;
	std::cout << "Stage Z position = " << mPosition3_mm[zz] << " mm" << std::endl;
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
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage Z to target position");

		mPosition3_mm[axis] = position_mm;
	}
}

//Move the 3 stages to the requested position
void Stage::moveStage3(const double3 positionXYZ_mm)
{
	moveStage(xx, positionXYZ_mm[xx]);
	moveStage(yy, positionXYZ_mm[yy]);
	moveStage(zz, positionXYZ_mm[zz]);
}


bool Stage::isMoving(const Axis axis) const
{
	BOOL isMoving = FALSE;

	if (!PI_IsMoving(mID[axis], mNstagesPerController, &isMoving))	//~55 ms to execute this function
		throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage " + std::to_string(axis));

	return isMoving;
}

void Stage::waitForMovementToStop(const Axis axis) const
{
	BOOL isMoving;
	do {
		if (!PI_IsMoving(mID[axis], mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + std::to_string(axis));

		std::cout << "#";
	} while (isMoving);

	std::cout << "\n";
}

void Stage::waitForMovementToStop3() const
{
	BOOL isMoving_x, isMoving_y, isMoving_z;
	do {
		if (!PI_IsMoving(mID[xx], mNstagesPerController, &isMoving_x))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage X");

		if (!PI_IsMoving(mID[yy], mNstagesPerController, &isMoving_y))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Y");

		if (!PI_IsMoving(mID[zz], mNstagesPerController, &isMoving_z))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage Z");

		std::cout << "#";
	} while (isMoving_x || isMoving_y || isMoving_z);

	std::cout << "\n";
}


//Convert from absolute tile number ---> (slice number, plane number, tile number)
// this function does NOT consider the overlaps
void Stage::scanningStrategy(const int nTileAbsolute) //Absolute tile number = 0, 1, 2, ..., total number of tiles
{
	int nTiles_x = 50;	//Number of tiles in x
	int nTiles_y = 50;	//Number of tiles in y
	int nTilesPerPlane = nTiles_x * nTiles_y; //Number of tiles in each plane
	int nPlanesPerSlice = 100;	//Number of planes in each slice
	int nSlice = 20; //Number of slices in the entire sample

	//	int nPlane;
	//std::vector<int> nTileXY;

	//total number of tiles = nSlice * nPlanesPerSlice * nTilesPerPlane

}


/*Pseudo code
SDx = +;
SDy = +;
SDz = -		//- is down, + is up

while{

if (reached zmax) //reached bottom of the slice
snakeXY();
SDz = +;

if (reached zmin)	//reached top of the slice
snakeXY();
SDz = -;
}


snakeXY:
if (SD = +)
if (!reached xmax)
x++;
else if (!reached ymax)
y++;
SDx = -; //change the scanning direction
else //reached xmax & ymax
nextSlice();

else //SDx = -
if (!reached xmin)
x--;
else if (!reach ymax)
y++;
SDx = +; //change the scanning direction
else	//reached xmin & ymax
nextSlice();


nextSlice:
if (!reached nMaxSlice)
runVibratome();
next slice();
else
stop;
*/


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


/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/