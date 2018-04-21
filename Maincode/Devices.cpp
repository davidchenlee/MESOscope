#include "Devices.h"

#pragma region "Photon counter"
int readPhotonCount(NiFpga_Status* status, NiFpga_Session session)
{
	//FIFOa
	int NelementsReadFIFOa = 0; 						//Total number of elements read from the FIFO
	U32 *dataFIFOa = new U32[NpixAllFrames];			//The buffer size does not necessarily have to be the size of a frame
														
	for (int ii = 0; ii < NpixAllFrames; ii++)			//Initialize the array for FIFOa
		dataFIFOa[ii] = 0;

	//FIFOb
	//Create an array of arrays to serve as a buffer and store the data from the FIFO
	//The ReadFifo function gives chuncks of data. Store each chunck in a separate buffer-array
	//I think I can't just make a long, concatenated 1D array because I have to pass individual arrays to the FIFO-read function
	int bufArrayIndexb = 0;								//Number of buffer arrays actually used
	const int NmaxbufArray = 100;
	U32 **bufArrayb = new U32*[NmaxbufArray];
	for (int i = 0; i < NmaxbufArray; i++)
		bufArrayb[i] = new U32[NpixAllFrames];			//Each row is used to store the data from the ReadFifo. The buffer size could possibly be < NpixAllFrames

	int *NelementsBufArrayb = new int[NmaxbufArray];	//Each elements in this array indicates the number of elements in each chunch of data
	int NelementsReadFIFOb = 0; 						//Total number of elements read from the FIFO


	//Start the FIFO OUT to transfer data from the FPGA FIFO to the PC FIFO
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

	//Trigger the acquisition. If triggered too early, the FPGA FIFO will probably overflow
	triggerFPGAstartImaging(status, session);

	//Read the data
	try
	{
		readFIFO(status, session, NelementsReadFIFOa, NelementsReadFIFOb, dataFIFOa, bufArrayb, NelementsBufArrayb, bufArrayIndexb, NmaxbufArray);

		//If all the expected data is read successfully, process the data
		if (NelementsReadFIFOa == NpixAllFrames && NelementsReadFIFOb == NpixAllFrames)
		{
			unsigned char *image = unpackFIFObuffer(bufArrayIndexb, NelementsBufArrayb, bufArrayb);
			correctInterleavedImage(image);
			writeFrameToTiff(image, "_photon-counts.tif");
			//writeFrameToTxt(image, "_photon-counts.txt");
			delete image;
		}
		else
			std::cerr << "ERROR in " << __func__ << ": more or less elements received from the FIFO than expected " << std::endl;
	}
	catch (const std::overflow_error& e) {
		// this executes if f() throws std::overflow_error (same type rule)
		std::cerr << e.what();
	}
	catch (const std::runtime_error& e) {
		// this executes if f() throws std::underflow_error (base class rule)
		std::cerr << e.what();
	}
	catch (const std::exception& e) {
		// this executes if f() throws std::logic_error (base class rule)
		std::cerr << e.what();
	}
	catch (...)
	{
		std::cerr << "Unknown exception Caught in " << __func__ << std::endl;
	}


	//Close the FIFO to (maybe) flush it
	NiFpga_MergeStatus(status, NiFpga_StopFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	NiFpga_MergeStatus(status, NiFpga_StopFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

	delete[] dataFIFOa;

	//clean up the buffer arrays
	for (int i = 0; i < NmaxbufArray; ++i) {
		delete[] bufArrayb[i];
	}
	delete[] bufArrayb;

	return 0;
}


void readFIFO(NiFpga_Status* status, NiFpga_Session session, int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray)
{
	const int readFifoWaitingTime = 15;			//Waiting time between each iteration
	const U32 timeout = 100;					//FIFO timeout
	U32 NremainingFIFOa, NremainingFIFOb;			//Elements remaining in the FIFO
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
	while (NelementsReadFIFOa < NpixAllFrames || NelementsReadFIFOb < NpixAllFrames)
	{
		Sleep(readFifoWaitingTime); //wait till collecting big chuncks of data. Adjust the waiting time until getting max transfer bandwidth

									//FIFO OUT a
		if (NelementsReadFIFOa < NpixAllFrames)
		{
			NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dummy, 0, timeout, &NremainingFIFOa));
			//std::cout << "Number of elements remaining in the host FIFO a: " << NremainingFIFOa << std::endl;

			if (NremainingFIFOa > 0)
			{
				NelementsReadFIFOa += NremainingFIFOa;

				NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dataFIFOa, NremainingFIFOa, timeout, &NremainingFIFOa));
			}
		}

		//FIFO OUT b
		if (NelementsReadFIFOb < NpixAllFrames)		//Skip if all the data have already been transferred (i.e. NelementsReadFIFOa = NpixAllFrames)
		{
			//By requesting 0 elements from the FIFO, the function returns the number of elements available. If no data available so far, then NremainingFIFOa = 0 is returned
			NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, dummy, 0, timeout, &NremainingFIFOb));
			//std::cout << "Number of elements remaining in the host FIFO b: " << NremainingFIFOb << std::endl;

			//If there are data available in the FIFO, retrieve it
			if (NremainingFIFOb > 0)
			{
				NelementsReadFIFOb += NremainingFIFOb;						//Keep track of the number of elements read so far
				NelementsBufArrayb[bufArrayIndexb] = NremainingFIFOb;		//Keep track of how many elements are in each FIFObuffer array												

																			//Read the elements in the FIFO
				NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, bufArrayb[bufArrayIndexb], NremainingFIFOb, timeout, &NremainingFIFOb));

				if (bufArrayIndexb >= NmaxbufArray)
				{
					throw std::range_error(std::string{} +"ERROR in " + __func__ + ": Buffer array overflow\n");
				}

				bufArrayIndexb++;
			}
		}

		timeoutCounter--;

		if (timeoutCounter == 0)					//Timeout the while loop in case the data transfer fails
		{
			std::cerr << "ERROR in " << __func__ << ": FIFO downloading timeout" << std::endl;
			break;
		}
	}

	//Stop the stopwatch
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Elapsed time: " << duration << " s" << std::endl;
	std::cout << "FIFO bandwidth: " << 2 * 32 * NpixAllFrames / duration / 1000000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer-arrays used: " << (U32)bufArrayIndexb << std::endl; //print how many buffer arrays were actually used
	std::cout << "Total of elements read: " << NelementsReadFIFOa << "\t" << NelementsReadFIFOb << std::endl; //print the total number of elements read
}

//Returns a single 1D array with the chucks of data stored in the buffer 2D array
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb)
{
	const bool debug = 0;												//For debugging. Generate numbers from 1 to NpixAllFrames with +1 increament

	static unsigned char *image = new unsigned char[NpixAllFrames];		//Create a long 1D array representing the image

	for (int ii = 0; ii < NpixAllFrames; ii++)							//Initialize the array
		image[ii] = 0;

	U32 pixIndex = 0;													//Index the image pixel
	for (int ii = 0; ii < bufArrayIndexb; ii++)
	{
		for (int jj = 0; jj < NelementsBufArrayb[ii]; jj++)
		{
			//myfile << bufArrayb[ii][jj] << std::endl;		
			image[pixIndex] = (unsigned char)bufArrayb[ii][jj];

			//For debugging. Generate numbers from 1 to NpixAllFrames with +1 increament
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
//One idea is to read bufArrayb line by line (1 line = Width_pix x 1) and save it to file using TIFFWriteScanline
int correctInterleavedImage(unsigned char *interleavedImage)
{
	unsigned char *auxLine = new unsigned char[WidthPerFrame_pix]; //one line to temp store the data. In principle I could just use half the size, but why bothering...

																   //for every odd-number line, reverse the pixel order
	for (int lineIndex = 1; lineIndex < HeightPerFrame_pix; lineIndex += 2)
	{
		//save the data in an aux array
		for (int pixIndex = 0; pixIndex < WidthPerFrame_pix; pixIndex++)
			auxLine[pixIndex] = interleavedImage[lineIndex*WidthPerFrame_pix + (WidthPerFrame_pix - pixIndex - 1)];
		//write the data back
		for (int pixIndex = 0; pixIndex < WidthPerFrame_pix; pixIndex++)
			interleavedImage[lineIndex*WidthPerFrame_pix + pixIndex] = auxLine[pixIndex];

	}
	delete[] auxLine;

	return 0;
}


int writeFrameToTxt(unsigned char *imageArray, std::string fileName)
{
	std::ofstream myfile;								//Create output file
	myfile.open(fileName);								//Open the file

	for (int ii = 0; ii < NpixAllFrames; ii++)
		myfile << (int)imageArray[ii] << std::endl;	//Write each element

	myfile.close();										//Close the txt file

	return 0;
}

#pragma endregion "Photon counter"


#pragma region "Vibratome"

Vibratome::Vibratome()
	: status(status),
	session(session)
{}

Vibratome::~Vibratome() {}

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
int Vibratome::setState()
{
	const int WaitingTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 1));
	Sleep(WaitingTime);
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));

	return 0;
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
int Vibratome::sendCommand(double pushDuration, VibratomeChannel channel)
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const int minPushDuration = 10; //in ms

	switch (channel)
	{
	case VibratomeBack:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_back;
		break;
	case VibratomeForward:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_forward;
		break;
	default:
		std::cerr << "ERROR in " << __func__ << ": Selected vibratome channel is unavailable" << std::endl;
		return -1;
	}


	const int delay = 1;	//Used to roughly calibrate the pulse length
	const int dt_ms = (int)pushDuration / ms;

	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 1));

	if (dt_ms >= minPushDuration)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minPushDuration - delay);
		std::cerr << "WARNING in " << __func__ << ": vibratome push duration too short. Instead, set to the min = ~" << minPushDuration << "ms" << std::endl;
	}
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 0));

	return 0;
}

#pragma endregion "Vibratome"

#pragma region "Resonant scanner"

ResonantScanner::ResonantScanner(NiFpga_Status* status, NiFpga_Session session)
	: status(status),
	session(session),
	voltPerUm(RS_voltPerUm),
	amplitude_um(0),
	delayTime(10)
{
//	this->status = status;
//	this->session = session;
//	voltPerUm = RS_voltPerUm;
//	amplitude_um = 0;
//	delayTime = 10;	//in ms
};
ResonantScanner::~ResonantScanner() {};

//Start or stop the resonant scanner
int ResonantScanner::setState(bool requestedState)
{
	this->state = requestedState;
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool)requestedState));

	return 0;
}

//Set the output voltage of the resonant scanner
int ResonantScanner::setOutputVoltage(double Vout)
{
	this->amplitude_um = Vout / this->voltPerUm;
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(Vout)));

	return 0;
}

//Set the output voltage of the resonant scanner
int ResonantScanner::setOutputAmplitude(double amplitude_um)
{
	this->amplitude_um = amplitude_um;
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(amplitude_um * voltPerUm)));

	return 0;
}

int ResonantScanner::turnOn(double amplitude_um)
{
	this->setState(0);	//make sure that disable is on
	Sleep(delayTime);
	this->setOutputAmplitude(amplitude_um);
	Sleep(delayTime);
	this->setState(1);

	return 0;
}

int ResonantScanner::turnOff()
{
	this->setState(0);
	Sleep(delayTime);
	this->setOutputVoltage(0);

	return 0;
}


double ResonantScanner::convertUm2Volt(double amplitude_um)
{
	return amplitude_um * this->voltPerUm;
}

#pragma endregion "Resonant scanner"

#pragma region "Shutters"

Shutter::Shutter(NiFpga_Status* status, NiFpga_Session session, uint32_t ID)
	: status(status),
	session(session),
	IDshutter(ID),
	delayTime(10)
{
//	this->status = status;
//	this->session = session;
//	IDshutter = ID;
//	delayTime = 10;	//in ms
}

Shutter::~Shutter() {}

int Shutter::setState(bool requestedState)
{
	state = requestedState;
	NiFpga_MergeStatus(this->status, NiFpga_WriteBool(this->session, IDshutter, (NiFpga_Bool)requestedState));

	return 0;
}

int Shutter::pulseHigh()
{
	if(state)	//make sure that the output is set LOW
		NiFpga_MergeStatus(this->status, NiFpga_WriteBool(this->session, IDshutter, 0));
	else
	{
		NiFpga_MergeStatus(this->status, NiFpga_WriteBool(this->session, IDshutter, 1));
		Sleep(delayTime);
		NiFpga_MergeStatus(this->status, NiFpga_WriteBool(this->session, IDshutter, 0));
	}
	return 0;
}

#pragma endregion "Shutters"

#pragma region "Pockels cells"
//Pockels cells
//NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));

#pragma endregion "Pockels cells"

#pragma region "Pixel clock"

PixelClock::PixelClock() {}

PixelClock::~PixelClock() {}

//Convert the spatial coordinate of the resonant scanner to time. x in [-RSamplitudePkPK_um/2, RSamplitudePkPK_um/2]
double PixelClock::ConvertSpatialCoord2Time(double x)
{
	const double arg = 2 * x / RSamplitudePkPK_um;
	if (arg <= 1)
		return HalfPeriodLineClock_us * asin(arg) / PI; //Return value in [-HalfPeriodLineClock_us/PI, HalfPeriodLineClock_us/PI]
	else
	{
		std::cerr << "ERROR in " << __func__ << ": argument of asin greater than 1" << std::endl;
		return HalfPeriodLineClock_us / PI;
	}
}

//Discretize the spatial coordinate, then convert it to time
double PixelClock::getDiscreteTime(int pix)
{
	const double dx = 0.5 * um;
	return ConvertSpatialCoord2Time(dx * pix);
}

//Calculate the dwell time for the pixel
double PixelClock::calculateDwellTime(int pix)
{
	return getDiscreteTime(pix + 1) - getDiscreteTime(pix);
}

//Calculate the dwell time of the pixel but considering that the FPGA has a finite clock rate
double PixelClock::calculatePracticalDwellTime(int pix)
{
	return round(calculateDwellTime(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}

//Pixel clock sequence. Every pixel has the same duration in time.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
//Pixel clock evently spaced in time
U32Q PixelClock::PixelClockEqualDuration()
{
	const double InitialWaitingTime_us = 6.25*us;						//Initial waiting time to center the pixel clock in a line scan
																		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
	Queue.push(packU32(convertUs2tick(InitialWaitingTime_us) - latency_tick, 0x0000));

	const double PixelTimeStep = 0.125 * us;
	for (int pix = 0; pix < WidthPerFrame_pix + 1; pix++)
		Queue.push(generateSinglePixelClock(PixelTimeStep, 1));			//Generate the pixel clock. Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter
																		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
	return Queue;														//Return a queue (and not a vector of queues)
}

//Pixel clock sequence. Every pixel is equally spaced.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
U32Q PixelClock::PixelClockEqualDistance()
{
	std::vector<double> PixelClockEqualDistanceLUT(WidthPerFrame_pix);

	if (WidthPerFrame_pix % 2 == 0)	//is even
	{
		for (int pix = -WidthPerFrame_pix / 2; pix < WidthPerFrame_pix / 2; pix++)	//pix in [-WidthPerFrame_pix/2,WidthPerFrame_pix/2]
			PixelClockEqualDistanceLUT[pix + WidthPerFrame_pix / 2] = calculatePracticalDwellTime(pix);
	}
	else
	{
		std::cerr << "ERROR in " << __func__ << ": Odd number of pixels in the image width currently not supported by the pixel clock. Pixel clock set to 0" << std::endl;
	}

	const U16 InitialWaitingTime_tick = 2043;										//Initial waiting time. Look at the oscilloscope and adjust this parameter to center the pixel clock in a line scan
	Queue.push(packU32(InitialWaitingTime_tick - latency_tick, 0x0000));

	for (int pix = 0; pix < WidthPerFrame_pix; pix++)
		Queue.push(generateSinglePixelClock(PixelClockEqualDistanceLUT[pix], 1));	//Generate the pixel clock.Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter

	Queue.push(generateSinglePixelClock(dt_us_MIN, 1));								//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant

	return Queue;
}

#pragma endregion "Pixel clock"
