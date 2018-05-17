#include "Devices.h"

#pragma region "Vibratome"

Vibratome::Vibratome(const FPGAapi &fpga): mFpga(fpga){}

Vibratome::~Vibratome() {}

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::startStop()
{
	const int SleepTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 1));

	Sleep(SleepTime);

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 0));
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
void Vibratome::sendCommand(const double pulseDuration, const VibratomeChannel channel)
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

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 1));

	if (dt_ms >= minPulseDuration)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minPulseDuration - delay);
		std::cerr << "WARNING in " << __FUNCTION__ << ": Vibratome pulse duration too short. Duration set to the min = ~" << minPulseDuration << "ms" << std::endl;
	}
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 0));
}

#pragma endregion "Vibratome"

#pragma region "Resonant scanner"

ResonantScanner::ResonantScanner(const FPGAapi &fpga): mFpga(fpga){};

ResonantScanner::~ResonantScanner() {};

//Start or stop the resonant scanner
void ResonantScanner::run(const bool state){
	
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool)state));
}


//Set the output voltage of the resonant scanner
void ResonantScanner::setVcontrol_volt(const double Vcontrol_volt)
{
	if (Vcontrol_volt > mVMAX_volt) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_volt) + " V" );

	mVcontrol_volt = Vcontrol_volt;
	mFFOV_um = Vcontrol_volt / mVoltPerUm;

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(mVcontrol_volt)));
}

//Set the output voltage of the resonant scanner
void ResonantScanner::setFFOV_um(const double FFOV_um)
{
	mVcontrol_volt = FFOV_um * mVoltPerUm;
	mFFOV_um = FFOV_um;

	if (mVcontrol_volt > mVMAX_volt) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_volt) + " V");

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(mVcontrol_volt)));
}

void ResonantScanner::turnOn_um(const double FFOV_um)
{
	setFFOV_um(FFOV_um);
	Sleep(mDelayTime_ms);
	run(1);
}

void ResonantScanner::turnOnSoft_volt(const double Vcontrol_volt)
{
	setVcontrol_volt(Vcontrol_volt);
	Sleep(mDelayTime_ms);
	run(1);
}

void ResonantScanner::turnOff()
{
	run(0);
	Sleep(mDelayTime_ms);
	setVcontrol_volt(0);
}


double ResonantScanner::convertUm2Volt(double amplitude_um)
{
	return amplitude_um * mVoltPerUm;
}

#pragma endregion "Resonant scanner"

#pragma region "Shutters"

Shutter::Shutter(const FPGAapi &fpga, ShutterID ID) : mFpga(fpga)
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

void Shutter::open()
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 1));
}

void Shutter::close()
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 0));
}

void Shutter::pulseHigh()
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 1));

	Sleep(mDelayTime_ms);

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mID, 0));
}

#pragma endregion "Shutters"

#pragma region "Pockels cells"
//PockelsID cells
//checkFPGAstatus(__FUNCTION__,  NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));

#pragma endregion "Pockels cells"


#pragma region "RTsequence"

RTsequence::RTsequence(const FPGAapi &fpga) : mFpga(fpga), mVectorOfQueues(Nchan)
{
	const Pixelclock pixelclock;
	mVectorOfQueues.at(PCLOCK) = pixelclock.readPixelclock();
}

RTsequence::~RTsequence() {}

QU32 RTsequence::generateLinearRamp(double TimeStep, const double RampLength, const double Vinitial, const double Vfinal)
{
	QU32 queue;
	const bool debug = 0;

	if (TimeStep < AOdt_us)
	{
		std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		TimeStep = AOdt_us;		//Analog output time increment (in us)
		return {};
	}

	const int nPoints = (int)(RampLength / TimeStep);		//Number of points

	if (nPoints <= 1)
	{
		std::cerr << "ERROR in " << __FUNCTION__ << ": Not enought points in the linear ramp" << std::endl;
		std::cerr << "nPoints: " << nPoints << std::endl;
		return {};
	}
	else
	{
		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}
		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V = Vinitial + (Vfinal - Vinitial)*ii / (nPoints - 1);
			queue.push_back(packSingleAnalog(TimeStep, V));

			if (debug)
				std::cout << (ii + 1) * TimeStep << "\t" << (ii + 1) * convertUs2tick(TimeStep) << "\t" << V << "\t" << std::endl;
		}
		if (debug)
		{
			getchar();
			return {};
		}
	}
	return queue;
}

void RTsequence::pushQueue(const RTchannel chan, QU32& queue)
{
	concatenateQueues(mVectorOfQueues.at(chan), queue);
}

void RTsequence::pushSingleValue(const RTchannel chan, const U32 input)
{
	mVectorOfQueues.at(chan).push_back(input);
}

void RTsequence::pushLinearRamp(const RTchannel chan, const double TimeStep, const double RampLength, const double Vinitial, const double Vfinal)
{
	concatenateQueues(mVectorOfQueues.at(chan), generateLinearRamp(TimeStep, RampLength, Vinitial, Vfinal));
}

//Push all the elements in 'tailQ' into 'headQ'
void RTsequence::concatenateQueues(QU32& receivingQueue, QU32& givingQueue)
{
	while (!givingQueue.empty())
	{
		receivingQueue.push_back(givingQueue.front());
		givingQueue.pop_front();
	}
}

//Upload the commands to the FPGA (see the implementation of the LV code), but do not execute yet
void  RTsequence::uploadRT()
{
	mFpga.writeFIFO(mVectorOfQueues);

	//On the FPGA, transfer the commands from FIFO IN to the sub-channel buffers
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
}

RTsequence::Pixelclock::Pixelclock()
{
	switch (pixelclockType) //pixelclockType defined globally
	{
	case equalDur: equalDuration();
		break;
	case equalDist: equalDistance();
		break;
	default: throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pixelclock type unavailable");
		break;
	}
}

RTsequence::Pixelclock::~Pixelclock() {}

//Convert the spatial coordinate of the resonant scanner to time. x in [-RSpkpk_um/2, RSpkpk_um/2]
double RTsequence::Pixelclock::ConvertSpatialCoord2Time_us(const double x)
{
	double arg = 2 * x / RSpkpk_um;
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Argument of asin greater than 1");
	else
		return halfPeriodLineclock_us * asin(arg) / Const::PI; //The returned value in in the range [-halfPeriodLineclock_us/PI, halfPeriodLineclock_us/PI]
}

//Discretize the spatial coordinate, then convert it to time
double RTsequence::Pixelclock::getDiscreteTime_us(const int pix)
{
	const double dx = 0.5 * um;
	return ConvertSpatialCoord2Time_us(dx * pix);
}

//Calculate the dwell time for the pixel
double RTsequence::Pixelclock::calculateDwellTime_us(const int pix)
{
	return getDiscreteTime_us(pix + 1) - getDiscreteTime_us(pix);
}

//Calculate the dwell time of the pixel but considering that the FPGA has a finite clock rate
double RTsequence::Pixelclock::calculatePracticalDwellTime_us(const int pix)
{
	return round(calculateDwellTime_us(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}


//Pixel clock sequence. Every pixel has the same duration in time.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
//Pixel clock evently spaced in time
void RTsequence::Pixelclock::equalDuration()
{
	//Relative delay of the pixel clock wrt the line clock (assuming perfect laser alignment, which is generally not true)
	//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
	const double initialWaitingTime_us = 6.25*us;							
	pixelclockQ.push_back(packU32(convertUs2tick(initialWaitingTime_us) - mLatency_tick, 0x0000));

	//Generate the pixel clock. When a HIGH is pushed, the pixel clock switches it state which represents a pixel delimiter
	//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
	const double dwellTime_us = 0.125 * us;
	for (int pix = 0; pix < widthPerFrame_pix + 1; pix++)
		pixelclockQ.push_back(packSinglePixelclock(dwellTime_us, 1));		
}

//Pixel clock sequence. Every pixel is equally spaced.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
void RTsequence::Pixelclock::equalDistance()
{
	if (widthPerFrame_pix % 2 != 0)	//Throw exception if odd. Odd number of pixels not supported yet
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Odd number of pixels in the image width currently not supported");

	//Relative delay of the pixel clock with respect to the line clock
	const U16 InitialWaitingTime_tick = (U16)(calibCoarse_tick + calibFine_tick);
	pixelclockQ.push_back(packU32(InitialWaitingTime_tick - mLatency_tick, 0x0000));

	//Generate the pixel clock. When a HIGH is pushed, the pixel clock switches it state which represents a pixel delimiter (the switching is implemented on the FPGA)
	for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)
		pixelclockQ.push_back(packSinglePixelclock(calculatePracticalDwellTime_us(pix), 1));

	//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
	pixelclockQ.push_back(packSinglePixelclock(dtMIN_us, 1));
}

QU32 RTsequence::Pixelclock::readPixelclock() const
{
	return pixelclockQ;
}

#pragma endregion "RTsequence"


#pragma region "Image"

Image::Image(const FPGAapi &fpga) : mFpga(fpga)
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

//The ReadFifo function gives chuncks of data. Store each chunck in a separate buffer-array. I think I can't just make a long, concatenated 1D array because I have to pass individual arrays to the FIFO-read function
void Image::acquire(const std::string filename)
{
	startFIFO();		//Start transferring data from the FPGA FIFO to the PC FIFO
	mFpga.triggerRT();	//Trigger the acquisition. If triggered too early, the FPGA FIFO will probably overflow

	try
	{
		readFIFO();			//Read the data in the FIFO and transfer it to the buffer
		unpackBuffer();		//Move the chuncks of data in the buffer to an array
		correctInterleavedImage();
		saveAsTiff(filename);
		//stopFIFOs();
	}
	catch (const ImageException &e) //Notify the exception and move on to the next iteration
	{
		stopFIFO();			//Close the FIFO to flush the data in the FIFO, otherwise segmentation fault accessing the FPGA and the computer will crash
		std::cout << "An ImageException has occurred in: " << e.what() << std::endl;
	}

}

void Image::startFIFO()
{
	checkFPGAstatus(__FUNCTION__, NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	checkFPGAstatus(__FUNCTION__, NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}

void Image::configureFIFO(const U32 depth)
{
	U32 actualDepth;
	checkFPGAstatus(__FUNCTION__, NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth));
	checkFPGAstatus(__FUNCTION__, NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "ActualDepth a: " << actualDepth << "\t" << "ActualDepth b: " << actualDepth << std::endl;
}

//TODO: save the data from the FIFO saving concurrently
//Read the PC-FIFO as the data arrive. I ran a test and found out that two 32-bit FIFOs has a larger bandwidth than a single 64 - bit FIFO
//Test if the bandwidth can be increased by using 'NiFpga_AcquireFifoReadElementsU32'.Ref: http://zone.ni.com/reference/en-XX/help/372928G-01/capi/functions_fifo_read_acquire/
//pass an array to a function: https://stackoverflow.com/questions/2838038/c-programming-malloc-inside-another-function
//review of pointers and references in C++: https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html
void Image::readFIFO()
{
	//Declare and start a stopwatch [2]
	double duration;
	auto t_start = std::chrono::high_resolution_clock::now();

	U32 *dummy = new U32[0];

	while (mNelemReadFIFO_A < nPixAllFrames || mNelemReadFIFO_B < nPixAllFrames)
	{
		Sleep(mReadFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time until getting max transfer bandwidth

		//FIFO OUT A
		if (mNelemReadFIFO_A < nPixAllFrames)
		{
			checkFPGAstatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dummy, 0, mTimeout_ms, &mNremainFIFO_A));
			//std::cout << "Number of elements remaining in the host FIFO a: " << nRemainFIFO_A << std::endl;

			if (mNremainFIFO_A > 0)
			{
				mNelemReadFIFO_A += mNremainFIFO_A;
				checkFPGAstatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, mBufArray_A, mNremainFIFO_A, mTimeout_ms, &mNremainFIFO_A));
			}
		}

		//FIFO OUT B
		if (mNelemReadFIFO_B < nPixAllFrames)		//Skip if all the data have already been transferred (i.e. nElemReadFIFO_A = nPixAllFrames)
		{
			//By requesting 0 elements from the FIFO, the function returns the number of elements available. If no data available so far, then nRemainFIFO_A = 0 is returned
			checkFPGAstatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, dummy, 0, mTimeout_ms, &mNremainFIFO_B));
			//std::cout << "Number of elements remaining in the host FIFO b: " << nRemainFIFO_B << std::endl;

			//If there are data available in the FIFO, retrieve it
			if (mNremainFIFO_B > 0)
			{
				mNelemReadFIFO_B += mNremainFIFO_B;							//Keep track of the number of elements read so far
				mNelemBufArray_B[mCounterBufArray_B] = mNremainFIFO_B;		//Keep track of how many elements are in each FIFObuffer array												

				//Read the elements in the FIFO
				checkFPGAstatus(__FUNCTION__, NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, mBufArray_B[mCounterBufArray_B], mNremainFIFO_B, mTimeout_ms, &mNremainFIFO_B));

				if (mCounterBufArray_B >= nBufArrays)
					throw ImageException((std::string)__FUNCTION__ + ": Buffer array overflow");

				mCounterBufArray_B++;
			}
		}

		mTimeoutCounter_iter--;

		//Transfer timeout
		if (mTimeoutCounter_iter == 0)
			throw ImageException((std::string)__FUNCTION__ + ": FIFO downloading timeout");
	}

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

	std::cout << "FIFO bandwidth: " << 2 * 32 * nPixAllFrames / duration / 1000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer-arrays used: " << (U32)mCounterBufArray_B << std::endl; //Print out how many buffer arrays were actually used
	std::cout << "Total of elements read: " << mNelemReadFIFO_A << "\t" << mNelemReadFIFO_B << std::endl; //Print out the total number of elements read

	//If expected data is read unsuccessfully
	if (mNelemReadFIFO_A != nPixAllFrames || mNelemReadFIFO_B != nPixAllFrames)
		throw ImageException((std::string)__FUNCTION__ + ": More or less FIFO elements received than expected ");

	delete[] dummy;
}

void Image::stopFIFO()
{
	checkFPGAstatus(__FUNCTION__, NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	checkFPGAstatus(__FUNCTION__, NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));
}


//When multiplexing later on, each U32 element in bufArray_B must be split in 8 parts of 4-bits each
//Returns a single 1D array with the chucks of data stored in the buffer 2D array
void Image::unpackBuffer()
{
	const bool debug = 0;
	double upscaledCount;

	U32 pixIndex = 0;	//Index for the image pixel
	for (int ii = 0; ii < mCounterBufArray_B; ii++)
	{
		for (int jj = 0; jj < mNelemBufArray_B[ii]; jj++)
		{
			upscaledCount = std::floor(upscaleU8 * mBufArray_B[ii][jj]); //Upscale the photoncount to a 8-bit number

			if (upscaledCount > _UI8_MAX)
				throw ImageException((std::string)__FUNCTION__ + ": Upscaled photoncount overflow");

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

void Image::saveAsTiff(std::string filename)
{
	if (!overrideImageSaving)
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
	fileHandle.open(foldername + filename + ".txt");						//Open the file

	for (int ii = 0; ii < nPixAllFrames; ii++)
		fileHandle << (int)image[ii] << std::endl;		//Write each element

	fileHandle.close();										//Close the txt file
}

//Check if the file already exists
std::string Image::file_exists(const std::string filename)
{
	std::string suffix = "";

	for (int ii = 1; std::experimental::filesystem::exists(foldername + filename + suffix + ".tif"); ii++)
		suffix = " (" + std::to_string(ii) + ")";

	return filename + suffix;
}

#pragma endregion "Image"

//Curently the output is hard coded on the FPGA side and triggered by the 'frame gate'
PockelsCell::PockelsCell(const FPGAapi &fpga, const PockelsID ID, const int wavelength_nm) : mFpga(fpga), mID(ID), mWavelength_nm(wavelength_nm)
{
	switch (ID)
	{
	case Pockels1:
		mFPGAvoltageControllerID = NiFpga_FPGAvi_ControlI16_PC1_voltage;
		mFPGAselectTriggerControllerID = NiFpga_FPGAvi_ControlBool_PC1_selectTrigger;
		mFPGAmanualOnControllerID = NiFpga_FPGAvi_ControlBool_PC1_manualOn;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}
}

//Do not set the output to 0 with the destructor to allow holding on the last value
PockelsCell::~PockelsCell() {}

void PockelsCell::setOutput_volt(const double V_volt)
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), mFPGAvoltageControllerID, convertVolt2I16(V_volt)));
	mV_volt = V_volt;
}

void PockelsCell::setOutput_mW(const double power_mW)
{
	double aux = convertPowertoVoltage_volt(power_mW);
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getSession(), mFPGAvoltageControllerID, convertVolt2I16(aux)));
	mV_volt = aux;
}


void PockelsCell::setOutputToZero()
{
	setOutput_volt(0);
}

//For debugging. Turn the pockels cell on without depending on the scanner
void PockelsCell::manualOn(const bool state)
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mFPGAselectTriggerControllerID, 1));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), mFPGAmanualOnControllerID, (NiFpga_Bool)state));
}


double PockelsCell::convertPowertoVoltage_volt(const double power_mW)
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

	//Delete "CHAMELEON>". This frase could be disabled on the laser but deleting it is more general and safer
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


#pragma region "Stages"
Stage::Stage()
{
	//Open the connections to the stage controllers and get assign their IDs
	mID[xx] = PI_ConnectUSB(mStageName_x.c_str());
	mID[yy] = PI_ConnectUSB(mStageName_y.c_str());
	mID[zz] = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 3 for "COM3" (CGS manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//mID[zz] = PI_ConnectUSB(mStageName_z.c_str());

	if (mID[xx] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage X");
	if (mID[yy] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Y");
	if (mID[zz] < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage Z");

	//Record the current position
	mPosition_mm[xx] = downloadPosition_mm(xx);
	mPosition_mm[yy] = downloadPosition_mm(yy);
	mPosition_mm[zz] = downloadPosition_mm(zz);
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
double3 Stage::recallPositionXYZ_mm() const
{
	return mPosition_mm;
}

void Stage::printPositionXYZ() const
{
	std::cout << "Stage X position = " << mPosition_mm[xx] << " mm" << std::endl;
	std::cout << "Stage Y position = " << mPosition_mm[yy] << " mm" << std::endl;
	std::cout << "Stage Z position = " << mPosition_mm[zz] << " mm" << std::endl;
}

//Retrieve the position from the stage
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
	if (position_mm < mPosMin_mm[axis] || position_mm > mPosMax_mm[axis])
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested position out of bounds for stage " + std::to_string(axis));

	//Move the stage
	if (mPosition_mm[axis] != position_mm ) //Move only if different position
	{
		if (!PI_MOV(mID[axis], mNstagesPerController, &position_mm) )	//~14 ms to execute this function
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to move stage Z to target position");

		mPosition_mm[axis] = position_mm;
	}
}

//Move the 3 stages to the requested position
void Stage::moveStage(const double3 positionXYZ_mm)
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

void Stage::waitForMovementStop(const Axis axis) const
{
	BOOL isMoving;
	do {
		if (!PI_IsMoving(mID[axis], mNstagesPerController, &isMoving))
			throw std::runtime_error((std::string)__FUNCTION__ + ": Unable to query movement status for stage" + std::to_string(axis));

		std::cout << "#";
	} while (isMoving);

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
double3 Stage::readAbsolutePosition_mm(const int nSlice, const int nPlane, const int3 nTileXY) const
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



Logger::Logger(const std::string filename)
{
	mFileHandle.open(foldername + filename + ".txt");
};

Logger::~Logger()
{
	mFileHandle.close();
};

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << std::endl;
}


/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (CGS manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)

[2] std::clock() vs std::chrono
http://en.cppreference.com/w/cpp/chrono/c/clock
*/