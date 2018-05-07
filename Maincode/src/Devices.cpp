#include "Devices.h"

#pragma region "Vibratome"

Vibratome::Vibratome(const FPGAapi &fpga): mFpga(fpga){}

Vibratome::~Vibratome() {}

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
void Vibratome::startStop()
{
	const int SleepTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	
	NiFpga_Status status = NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 1);
	checkFPGAstatus(__FUNCTION__, status);

	Sleep(SleepTime);

	status = NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_VT_start, 0);
	checkFPGAstatus(__FUNCTION__, status);
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
void Vibratome::sendCommand(const double pulseDuration, const VibratomeChannel channel)
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const int minPulseDuration = 10; //in ms
	const int delay = 1;	//Used to roughly calibrate the pulse length
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

	NiFpga_Status status = NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 1);
	checkFPGAstatus(__FUNCTION__, status);

	if (dt_ms >= minPulseDuration)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minPulseDuration - delay);
		std::cerr << "WARNING in " << __FUNCTION__ << ": vibratome pulse duration too short. Duration set to the min = ~" << minPulseDuration << "ms" << std::endl;
	}
	status = NiFpga_WriteBool(mFpga.getSession(), selectedChannel, 0);
	checkFPGAstatus(__FUNCTION__, status);
}

#pragma endregion "Vibratome"

#pragma region "Resonant scanner"

ResonantScanner::ResonantScanner(const FPGAapi &fpga): mFpga(fpga){};

ResonantScanner::~ResonantScanner() {};

//Start or stop the resonant scanner
void ResonantScanner::run(const bool state){
	
	NiFpga_Status status =  NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool)state);
	checkFPGAstatus(__FUNCTION__, status);
}


//Set the output voltage of the resonant scanner
void ResonantScanner::setVcontrol_volt(const double Vcontrol_volt)
{
	if (Vcontrol_volt > mVMAX_volt) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_volt) + " V" );

	mVcontrol_volt = Vcontrol_volt;
	mFFOV_um = Vcontrol_volt / mVoltPerUm;

	NiFpga_Status status = NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(mVcontrol_volt));
	checkFPGAstatus(__FUNCTION__, status);
}

//Set the output voltage of the resonant scanner
void ResonantScanner::setFFOV_um(const double FFOV_um)
{
	mVcontrol_volt = FFOV_um * mVoltPerUm;
	mFFOV_um = FFOV_um;

	if (mVcontrol_volt > mVMAX_volt) throw std::invalid_argument((std::string)__FUNCTION__ + ": Requested voltage greater than " + std::to_string(mVMAX_volt) + " V");

	NiFpga_Status status = NiFpga_WriteI16(mFpga.getSession(), NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(mVcontrol_volt));
	checkFPGAstatus(__FUNCTION__, status);
}

void ResonantScanner::turnOn_um(const double FFOV_um)
{
	setFFOV_um(FFOV_um);
	Sleep(mDelayTime);
	run(1);
}

void ResonantScanner::turnOn_volt(const double Vcontrol_volt)
{
	setVcontrol_volt(Vcontrol_volt);
	Sleep(mDelayTime);
	run(1);
}

void ResonantScanner::turnOff()
{
	run(0);
	Sleep(mDelayTime);
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
	NiFpga_Status status =  NiFpga_WriteBool(mFpga.getSession(), mID, 1);
	checkFPGAstatus(__FUNCTION__, status);
}

void Shutter::close()
{
	NiFpga_Status status = NiFpga_WriteBool(mFpga.getSession(), mID, 0);
	checkFPGAstatus(__FUNCTION__, status);
}

void Shutter::pulseHigh()
{
	NiFpga_Status status =  NiFpga_WriteBool(mFpga.getSession(), mID, 1);
	checkFPGAstatus(__FUNCTION__, status);

	Sleep(mDelayTime);

	status = NiFpga_WriteBool(mFpga.getSession(), mID, 0);
	checkFPGAstatus(__FUNCTION__, status);
}

#pragma endregion "Shutters"

#pragma region "Pockels cells"
//PockelsID cells
//NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));

#pragma endregion "Pockels cells"


#pragma region "RTsequence"

RTsequence::RTsequence(const FPGAapi &fpga) : mFpga(fpga), mVectorOfQueues(Nchan)
{
	PixelClock pixelclock;

	//mVectorOfQueues[PCLOCK]= pixelclock.PixelClockEqualDuration();
	mVectorOfQueues[PCLOCK] = pixelclock.PixelClockEqualDistance();

	dataFIFO_A = new U32[nPixAllFrames]();
	nElemBufArray_B = new int[nBufArrays]();

	bufArray_B = new U32*[nBufArrays];
	for (int i = 0; i < nBufArrays; i++)
		bufArray_B[i] = new U32[nPixAllFrames]();
}

RTsequence::~RTsequence()
{
	delete[] dataFIFO_A;
	delete[] nElemBufArray_B;

	//clean up the buffer arrays
	for (int i = 0; i < nBufArrays; ++i) {
		delete[] bufArray_B[i];
	}
	delete[] bufArray_B;

	//std::cout << "RT destructor was called" << std::endl;
}

QU32 RTsequence::generateLinearRamp(double TimeStep, const double RampLength, const double Vinitial, const double Vfinal)
{
	QU32 queue;
	const bool debug = 0;

	if (TimeStep < AOdt_us)
	{
		std::cerr << "WARNING in " << __FUNCTION__ << ": time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		TimeStep = AOdt_us;						//Analog output time increment (in us)
		return {};
	}

	const int nPoints = (int)(RampLength / TimeStep);		//Number of points

	if (nPoints <= 1)
	{
		std::cerr << "ERROR in " << __FUNCTION__ << ": not enought points for the linear ramp" << std::endl;
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
			queue.push(singleAnalogOut(TimeStep, V));

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
	concatenateQueues(mVectorOfQueues[chan], queue);
}

void RTsequence::pushSingleValue(const RTchannel chan, const U32 input)
{
	mVectorOfQueues[chan].push(input);
}

void RTsequence::pushLinearRamp(const RTchannel chan, const double TimeStep, const double RampLength, const double Vinitial, const double Vfinal)
{
	concatenateQueues(mVectorOfQueues[chan], generateLinearRamp(TimeStep, RampLength, Vinitial, Vfinal));
}

//Push all the elements in 'tailQ' into 'headQ'
void RTsequence::concatenateQueues(QU32& receivingQueue, QU32& givingQueue)
{
	while (!givingQueue.empty())
	{
		receivingQueue.push(givingQueue.front());
		givingQueue.pop();
	}
}

//Distribute the commands among the different channels (see the implementation of the LV code), but do not execute yet
void  RTsequence::loadRTsequenceonFPGA()
{
	mFpga.writeFIFO(mVectorOfQueues);

	NiFpga_Status status = NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1);
	checkFPGAstatus(__FUNCTION__, status);

	status = NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0);
	checkFPGAstatus(__FUNCTION__, status);

	//std::cout << "Pulse trigger status: " << mStatus << std::endl;
}

RTsequence::PixelClock::PixelClock() {}

RTsequence::PixelClock::~PixelClock() {}

//Convert the spatial coordinate of the resonant scanner to time. x in [-RSpkpk_um/2, RSpkpk_um/2]
double RTsequence::PixelClock::ConvertSpatialCoord2Time(const double x)
{
	double arg = 2 * x / RSpkpk_um;
	if (arg > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Argument of asin greater than 1");
	else
		return halfPeriodLineClock_us * asin(arg) / PI; //Return value in [-halfPeriodLineClock_us/PI, halfPeriodLineClock_us/PI]
}

//Discretize the spatial coordinate, then convert it to time
double RTsequence::PixelClock::getDiscreteTime(const int pix)
{
	const double dx = 0.5 * um;
	return ConvertSpatialCoord2Time(dx * pix);
}

//Calculate the dwell time for the pixel
double RTsequence::PixelClock::calculateDwellTime(const int pix)
{
	return getDiscreteTime(pix + 1) - getDiscreteTime(pix);
}

//Calculate the dwell time of the pixel but considering that the FPGA has a finite clock rate
double RTsequence::PixelClock::calculatePracticalDwellTime(const int pix)
{
	return round(calculateDwellTime(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}


//Pixel clock sequence. Every pixel has the same duration in time.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
//Pixel clock evently spaced in time
QU32 RTsequence::PixelClock::PixelClockEqualDuration()
{
	QU32 queue;
	const double InitialTimeStep_us = 6.25*us;							//Relative delay of the pixel clock wrt the line clock (assuming perfect laser alignment, which is generally not true)
																		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
	queue.push(packU32(convertUs2tick(InitialTimeStep_us) - mLatency_tick, 0x0000));

	const double PixelTimeStep = 0.125 * us;
	for (int pix = 0; pix < widthPerFrame_pix + 1; pix++)
		queue.push(singlePixelClock(PixelTimeStep, 1));			//Generate the pixel clock. Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter
																//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
	return queue;
}

//Pixel clock sequence. Every pixel is equally spaced.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
QU32 RTsequence::PixelClock::PixelClockEqualDistance()
{
	QU32 queue;
	std::vector<double> PixelClockEqualDistanceLUT(widthPerFrame_pix);

	if (widthPerFrame_pix % 2 != 0)	//is Odd. Odd number of pixels not supported yet
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Odd number of pixels for the image width currently not supported");

	for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)	//pix in [-widthPerFrame_pix/2,widthPerFrame_pix/2]
		PixelClockEqualDistanceLUT[pix + widthPerFrame_pix / 2] = calculatePracticalDwellTime(pix);

	const U16 InitialTimeStep_tick = (U16)(calibCoarse_tick + calibFine_tick);		//relative delay of the pixel clock wrt the line clock
	queue.push(packU32(InitialTimeStep_tick - mLatency_tick, 0x0000));

	for (int pix = 0; pix < widthPerFrame_pix; pix++)
		queue.push(singlePixelClock(PixelClockEqualDistanceLUT[pix], 1));	//Generate the pixel clock.Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter

	queue.push(singlePixelClock(dt_us_MIN, 1));								//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant

	return queue;
}

//Create an array of arrays to serve as a buffer and store the data from the FIFO
//The ReadFifo function gives chuncks of data. Store each chunck in a separate buffer-array
//I think I can't just make a long, concatenated 1D array because I have to pass individual arrays to the FIFO-read function
void RTsequence::runRTsequence()
{
	startFIFOs();				//Start the FIFO OUT to transfer data from the FPGA FIFO to the PC FIFO
	mFpga.runRTsequence();		//Trigger the acquisition. If triggered too early, the FPGA FIFO will probably overflow
	readFIFO();					//Read the data

	//If NOT all the expected data is read successfully
	if (nElemReadFIFO_A != nPixAllFrames || nElemReadFIFO_B != nPixAllFrames)
	{
		throw std::runtime_error((std::string)__FUNCTION__ + ": More or less elements received from the FIFO than expected ");
		//std::cerr << "ERROR in " << __FUNCTION__ << ": more or less elements received from the FIFO than expected " << std::endl;
	}
	else
	{
		unsigned char *image = unpackFIFObuffer(counterBufArray_B, nElemBufArray_B, bufArray_B);
		correctInterleavedImage(image);
		writeFrametoTiff(image, "_photon-counts.tif");
		//writeFrametoTxt(image, "_photon-counts.txt");
		delete image;
	}

	
	

	stopFIFOs();				//Close the FIFO to (maybe) flush it
}

void RTsequence::startFIFOs()
{
	NiFpga_Status status = NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa);
	checkFPGAstatus(__FUNCTION__, status);
	status = NiFpga_StartFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb);
	checkFPGAstatus(__FUNCTION__, status);
}

void RTsequence::readFIFO()
{
	const int readFifoWaitingTime = 15;			//Waiting time between each iteration
	const U32 timeout = 100;					//FIFO timeout
	U32 nRemainFIFO_A = 0;
	U32 nRemainFIFO_B = 0;			//Elements remaining in the FIFO
	int timeoutCounter = 100;					//Timeout the while-loop if the data-transfer from the FIFO fails	

	//Declare and start a stopwatch
	std::clock_t start;
	double duration;
	start = std::clock();

	//TODO: save the data from the FIFO saving concurrently
	//Read the PC-FIFO as the data arrive. I ran a test and found out that two 32-bit FIFOs has a larger bandwidth than a single 64 - bit FIFO
	//Test if the bandwidth can be increased by using 'NiFpga_AcquireFifoReadElementsU32'.Ref: http://zone.ni.com/reference/en-XX/help/372928G-01/capi/functions_fifo_read_acquire/
	//pass an array to a function: https://stackoverflow.com/questions/2838038/c-programming-malloc-inside-another-function
	//review of pointers and references in C++: https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html
	U32 *dummy = new U32[0];
	NiFpga_Status status;
	while (nElemReadFIFO_A < nPixAllFrames || nElemReadFIFO_B < nPixAllFrames)
	{
		Sleep(readFifoWaitingTime); //wait till collecting big chuncks of data. Adjust the waiting time until getting max transfer bandwidth

		//FIFO OUT A
		if (nElemReadFIFO_A < nPixAllFrames)
		{
			status = NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dummy, 0, timeout, &nRemainFIFO_A);//////////////////////////////////////
			checkFPGAstatus(__FUNCTION__, status);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//std::cout << "Number of elements remaining in the host FIFO a: " << nRemainFIFO_A << std::endl;

			if (nRemainFIFO_A > 0)
			{
				nElemReadFIFO_A += nRemainFIFO_A;

				status = NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dataFIFO_A, nRemainFIFO_A, timeout, &nRemainFIFO_A);/////////////////
				checkFPGAstatus(__FUNCTION__, status);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			}
		}

		//FIFO OUT B
		if (nElemReadFIFO_B < nPixAllFrames)		//Skip if all the data have already been transferred (i.e. nElemReadFIFO_A = nPixAllFrames)
		{
			//By requesting 0 elements from the FIFO, the function returns the number of elements available. If no data available so far, then nRemainFIFO_A = 0 is returned
			status = NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, dummy, 0, timeout, &nRemainFIFO_B);//////////////////////////////////////
			checkFPGAstatus(__FUNCTION__, status);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//std::cout << "Number of elements remaining in the host FIFO b: " << nRemainFIFO_B << std::endl;

			//If there are data available in the FIFO, retrieve it
			if (nRemainFIFO_B > 0)
			{
				nElemReadFIFO_B += nRemainFIFO_B;						//Keep track of the number of elements read so far
				nElemBufArray_B[counterBufArray_B] = nRemainFIFO_B;		//Keep track of how many elements are in each FIFObuffer array												

				//Read the elements in the FIFO
				status = NiFpga_ReadFifoU32(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, bufArray_B[counterBufArray_B], nRemainFIFO_B, timeout, &nRemainFIFO_B);//////////////////////////////////////
				checkFPGAstatus(__FUNCTION__, status);///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				if (counterBufArray_B >= nBufArrays)
					throw std::range_error(std::string{} +"ERROR in " + __FUNCTION__ + ": Buffer array overflow\n");

				counterBufArray_B++;
			}
		}

		timeoutCounter--;
		
		
		if (timeoutCounter == 0)	//Timeout the data transfer
			throw std::runtime_error((std::string)__FUNCTION__ + ": FIFO downloading timeout");
	

		/*
		if (timeoutCounter == 0) {
			std::cerr << "ERROR in " << __FUNCTION__ << ": FIFO downloading timeout" << std::endl;
			break;
		}
		*/
		
		
	}

	//Stop the stopwatch
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Elapsed time: " << duration << " s" << std::endl;
	std::cout << "FIFO bandwidth: " << 2 * 32 * nPixAllFrames / duration / 1000000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer-arrays used: " << (U32)counterBufArray_B << std::endl; //print how many buffer arrays were actually used
	std::cout << "Total of elements read: " << nElemReadFIFO_A << "\t" << nElemReadFIFO_B << std::endl; //print the total number of elements read
}

void RTsequence::configureFIFO(U32 depth)
{
	U32 actualDepth;
	NiFpga_Status status = NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth);
	checkFPGAstatus(__FUNCTION__, status);
	
	status = NiFpga_ConfigureFifo2(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth);
	checkFPGAstatus(__FUNCTION__, status);

	std::cout << "actualDepth a: " << actualDepth << std::endl;
	std::cout << "actualDepth b: " << actualDepth << std::endl;
}

void RTsequence::stopFIFOs()
{
	NiFpga_Status status = NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa);
	checkFPGAstatus(__FUNCTION__, status);
	status = NiFpga_StopFifo(mFpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb);
	checkFPGAstatus(__FUNCTION__, status);
}

#pragma endregion "RTsequence"

//Returns a single 1D array with the chucks of data stored in the buffer 2D array
unsigned char *unpackFIFObuffer(const int counterBufArray_B, int *nElemBufArray_B, U32 **bufArray_B)
{
	const bool debug = 0;												//For debugging. Generate numbers from 1 to nPixAllFrames with +1 increament

	static unsigned char *image = new unsigned char[nPixAllFrames];		//Create a long 1D array representing the image

	for (int ii = 0; ii < nPixAllFrames; ii++)							//Initialize the array
		image[ii] = 0;

	U32 pixIndex = 0;													//Index the image pixel
	for (int ii = 0; ii < counterBufArray_B; ii++)
	{
		for (int jj = 0; jj < nElemBufArray_B[ii]; jj++)
		{
			//myfile << bufArray_B[ii][jj] << std::endl;		
			image[pixIndex] = (unsigned char)bufArray_B[ii][jj];

			//For debugging. Generate numbers from 1 to nPixAllFrames with +1 increament
			if (debug)
			{
				image[pixIndex] = pixIndex + 1;
			}
			pixIndex++;
		}
	}
	return image;
}


//The microscope scans bidirectionally. The pixel order is backwards every other line.
//Later on, write the tiff directly from the buffer arrays. To deal with segmented pointers, use memcpy, memset, memmove or the Tiff versions for such functions
//memset http://www.cplusplus.com/reference/cstring/memset/
//memmove http://www.cplusplus.com/reference/cstring/memmove/
//One idea is to read bufArray_B line by line (1 line = Width_pix x 1) and save it to file using TIFFWriteScanline
int correctInterleavedImage(unsigned char *interleavedImage)
{
	unsigned char *auxLine = new unsigned char[widthPerFrame_pix]; //one line to temp store the data. In principle I could just use half the size, but why bothering...

																   //for every odd-number line, reverse the pixel order
	for (int lineIndex = 1; lineIndex < heightPerFrame_pix; lineIndex += 2)
	{
		//save the data in an aux array
		for (int pixIndex = 0; pixIndex < widthPerFrame_pix; pixIndex++)
			auxLine[pixIndex] = interleavedImage[lineIndex*widthPerFrame_pix + (widthPerFrame_pix - pixIndex - 1)];
		//write the data back
		for (int pixIndex = 0; pixIndex < widthPerFrame_pix; pixIndex++)
			interleavedImage[lineIndex*widthPerFrame_pix + pixIndex] = auxLine[pixIndex];

	}
	delete[] auxLine;

	return 0;
}


int writeFrametoTxt(unsigned char *imageArray, const std::string fileName)
{
	std::ofstream myfile;								//Create output file
	myfile.open(fileName);								//Open the file

	for (int ii = 0; ii < nPixAllFrames; ii++)
		myfile << (int)imageArray[ii] << std::endl;	//Write each element

	myfile.close();										//Close the txt file

	return 0;
}

PockelsCell::PockelsCell(const FPGAapi &fpga, const PockelsID ID, const int wavelength_nm) : mFpga(fpga), mID(ID), mWavelength_nm(wavelength_nm)
{
	switch (ID)
	{
	case Pockels1:
		mFPGAid = NiFpga_FPGAvi_ControlI16_PC1_voltage;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pockels cell unavailable");
	}
}

PockelsCell::~PockelsCell() {}

//before, the output power was max at 0V and the voltage for the min power depended on the wavelength as
double PockelsCell::voltageforMinPower()
{
	return 0.00361 * mWavelength_nm - 0.206;
}


void PockelsCell::turnOn_volt(const double V_volt)
{
	mV_volt = V_volt;

	NiFpga_Status status = NiFpga_WriteI16(mFpga.getSession(), mFPGAid, convertVolt2I16(mV_volt));
	checkFPGAstatus(__FUNCTION__, status);
}

void PockelsCell::turnOn_mW(const double power_mW)
{
	mV_volt = convertPowertoVoltage_volt(power_mW);

	NiFpga_Status status = NiFpga_WriteI16(mFpga.getSession(), mFPGAid, convertVolt2I16(mV_volt));
	checkFPGAstatus(__FUNCTION__, status);
}

void PockelsCell::turnOff()
{
	NiFpga_Status status = NiFpga_WriteI16(mFpga.getSession(), mFPGAid, 0);
	checkFPGAstatus(__FUNCTION__, status);

	mV_volt = 0;
}


double PockelsCell::convertPowertoVoltage_volt(double power_mW)
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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": laser wavelength " + std::to_string(mWavelength_nm) + " nm currently not calibrated");

	double arg = sqrt(power_mW / a);
	if (arg > 1) throw std::invalid_argument((std::string)__FUNCTION__ + ": arg of asin is greater than 1");

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
	std::string RxBuffer;				//Reply from the filterwheel
	const int RxBufSize = 10;

	size_t bytesWrote = mSerial->write(TxBuffer);
	size_t bytesRead = mSerial->read(RxBuffer, RxBufSize);

	//Delete echoed command. Echoing could be disabled on the laser but deleting it is more general and safer
	std::string::size_type i = RxBuffer.find(TxBuffer);
	if (i != std::string::npos)
		RxBuffer.erase(i, TxBuffer.length());

	//Delete CR and >
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
	//RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());

	mPosition = static_cast<FilterColor>(std::atoi(RxBuffer.c_str()));	//convert string to int, then to FilterColor
	//std::cout << RxBuffer << std::endl;
}

FilterColor Filterwheel::readFilterPosition()
{
	return mPosition;
}

void Filterwheel::setFilterPosition(const FilterColor color)
{
	if (color != mPosition)
	{
		std::string TxBuffer = "pos=" + std::to_string(color) + "\r";
		size_t bytesWrote = mSerial->write(TxBuffer);
		mPosition = color;
	}
}

Laser::Laser()
{
	mSerial = new serial::Serial(port, mBaud, serial::Timeout::simpleTimeout(mTimeout_ms));
	this->readWavelength_();
};

Laser::~Laser() {};

void Laser::readWavelength_()
{
	const std::string TxBuffer = "?VW";		//Command to the filterwheel
	std::string RxBuffer;						//Reply from the filterwheel
	const int RxBufSize = 20;

	size_t bytesWrote = mSerial->write(TxBuffer + "\r");
	size_t bytesRead = mSerial->read(RxBuffer, RxBufSize);

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

int Laser::readWavelength()
{
	return mWavelength;
}

void Laser::setWavelength()
{
	const std::string TxBuffer = "VW=800";		//Command to the filterwheel
	std::string RxBuffer;						//Reply from the filterwheel
	const int RxBufSize = 256;

	size_t bytesWrote = mSerial->write(TxBuffer + "\r");
	size_t bytesRead = mSerial->read(RxBuffer, RxBufSize);

	this->readWavelength_();
}


#pragma region "Stages"
Stage::Stage()
{
	//TODO: open the stages concurrently
	mID_x = PI_ConnectUSB(mStageName_x.c_str());
	//std::cout << "X: " << mID_x << std::endl;

	mID_y = PI_ConnectUSB(mStageName_y.c_str());
	//std::cout << "Y: " << mID_y << std::endl;

	//ZstageID = PI_ConnectUSB(mStageName_z.c_str());
	mID_z = PI_ConnectRS232(mPort_z, mBaud_z); // nPortNr = 3 for "COM3" (manual p12). For some reason 'PI_ConnectRS232' connects faster than 'PI_ConnectUSB'. More comments in [1]
	//std::cout << "Z: " << mID_z << std::endl;

	if (mID_x  < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage " + std::to_string(mID_x));
	if (mID_y  < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage " + std::to_string(mID_y));
	if (mID_z  < 0) throw std::runtime_error((std::string)__FUNCTION__ + ": Could not connect to the stage " + std::to_string(mID_z));

	mAbsPosition_mm.at(0)  = readPosition_mm_(mID_x);
	mAbsPosition_mm.at(1)  = readPosition_mm_(mID_y);
	mAbsPosition_mm.at(2)  = readPosition_mm_(mID_z);
}

Stage::~Stage()
{
	// Close Connections		
	PI_CloseConnection(mID_x);
	PI_CloseConnection(mID_y);
	PI_CloseConnection(mID_z);
	std::cout << "Stages connection closed.\n";
}

double Stage::readPosition_mm_(int ID)
{
	double position_mm;
	if (!PI_qPOS(ID, mNAxes, &position_mm)) throw std::runtime_error((std::string)__FUNCTION__  ": Unable to query position for the stage " + std::to_string(ID));

	return position_mm;
}

dXYZ Stage::readPosition_mm()
{
	return mAbsPosition_mm;
}

void Stage::printPosition()
{
	std::cout << "Stage " << mID_x << " position = " << mAbsPosition_mm.at(0) << " mm" << std::endl;
	std::cout << "Stage " << mID_y << " position = " << mAbsPosition_mm.at(1) << " mm" << std::endl;
	std::cout << "Stage " << mID_z << " position = " << mAbsPosition_mm.at(2) << " mm" << std::endl;
}

void Stage::moveToPosition_mm()
{
	
}

// Additional Sample Functions
int XstageID, YstageID, ZstageID;
char NumberOfAxesPerController[] = "1"; //There is only 1 stage per controller


// Determine boundaries for stage movement
bool GetStageBondaries(int stageID)
{
	double MinPositionValue, MaxPositionValue;
	if (!PI_qTMN(stageID, NumberOfAxesPerController, &MinPositionValue))
	{
		CloseConnectionWithComment(stageID, "TMN? unable to query min. position of axis.\n");
		return false;
	}

	if (!PI_qTMX(stageID, NumberOfAxesPerController, &MaxPositionValue))
	{
		CloseConnectionWithComment(stageID, "TMX?, Unable to query max. position of axis.\n");
		return false;
	}

	std::cout << "Allowed range of movement: min: " << MinPositionValue << "\t max: " << MaxPositionValue << "\n";
	return true;
}

bool ReferenceIfNeeded(int PIdeviceId, char* axis)
{
	BOOL Referenced;
	BOOL Flag;
	if (!PI_qFRF(PIdeviceId, axis, &Referenced))
		return false;

	// If stage is equipped with absolute sensors, Referenced will always be set to true.
	if (!Referenced)
	{
		std::cout << "Referencing axis " << axis << "\n";
		if (!PI_FRF(PIdeviceId, axis))
		{
			return false;
		}

		// Wait until the reference move is done.
		Flag = FALSE;
		while (Flag != TRUE)
		{
			if (!PI_IsControllerReady(PIdeviceId, &Flag))
				return false;
		}
	}
	return true;
}


void CloseConnectionWithComment(int PIdeviceId, const char* comment)
{
	std::cout << comment << "\n";
	ReportError(PIdeviceId);
	PI_CloseConnection(PIdeviceId);
}

void ReportError(int PIdeviceId)
{
	int err = PI_GetError(PIdeviceId);
	char zErrMsg[300];

	if (PI_TranslateError(err, zErrMsg, 299))
		std::cout << "Error " << err << " occurred: " << zErrMsg << "\n";
}

bool referenceStageZ()
{
	// Switch on servo
	const BOOL ServoOn = true;
	if (!PI_SVO(ZstageID, NumberOfAxesPerController, &ServoOn))
	{
		CloseConnectionWithComment(ZstageID, "SVO failed. Exiting.\n");
		return false;
	}

	// Reference stage
	if (!ReferenceIfNeeded(ZstageID, NumberOfAxesPerController))
	{
		CloseConnectionWithComment(ZstageID, "Not referenced, Referencing failed.\n");
		return false;
	}

	// Check if referencing was successful
	BOOL referenceCompleted;
	referenceCompleted = false;

	if (!PI_qFRF(ZstageID, NumberOfAxesPerController, &referenceCompleted))
	{
		CloseConnectionWithComment(ZstageID, "Failed to query reference status.\n");
		return false;
	}

	// Abort execution if stage could not be referenced
	if (false == referenceCompleted)
	{
		CloseConnectionWithComment(ZstageID, "Referencing failed.\n");
		return false;
	}

	std::cout << "Stage Z is referenced.\n";
	return true;
}
#pragma endregion "Stages"

//Convert from absolute tile number ---> (slice number, plane number, tile number)
// this function does NOT consider the overlaps
void Stage::scanningStrategy(int nTileAbsolute) //Absolute tile number = 0, 1, 2, ..., total number of tiles
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
dXYZ Stage::readAbsolutePosition_mm(int nSlice, int nPlane, iXYZ nTileXY)
{
	const double mm = 1;
	const double um = 0.001;

	dXYZ absPosition_mm {};
	dXYZ initialPosition_mm { 31.9*mm, 9.5*mm, 18.546*mm };
	dXYZ overlap_um { 20.*um, 20.*um, 30.*um };
	dXYZ FFOVxy_um { 200.*um, 200.*um };						//Full FOV

	double sliceThickness_um = 100 * um;
	double stepZ_um = 1;

	absPosition_mm[0] = initialPosition_mm[0] + nTileXY[0] * (FFOVxy_um[0] - overlap_um[0]);
	absPosition_mm[1] = initialPosition_mm[1] + nTileXY[1] * (FFOVxy_um[1] - overlap_um[1]);
	absPosition_mm[2] = initialPosition_mm[2] - nSlice * (sliceThickness_um - overlap_um[2]) - nPlane * stepZ_um;

	return absPosition_mm;
}
/*
[1] The stage Z has a virtual COM port that works on top of the USB connection (manual p9). This is, the function PI_ConnectRS232(int nPortNr, int iBaudRate) can be used even when the controller (Mercury C-863) is connected via USB.
nPortNr: to know the correct COM port, look at Window's device manager or use Tera Term. Use nPortNr=1 for COM1, etc..
iBaudRate: the manual says that the baud rate does not matter (p10), but the suggested 115200 does not work. I use the default baud rate = 38400 which matches the drive's front panel configuration (using physical switches)
*/