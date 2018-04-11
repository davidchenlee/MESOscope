#include "FPGAlowlevel.h"

#pragma region "FPGA low-level functions"

void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}


//time t and analog output x are encoded in 16 bits each. Pack t in MSB and x in LSB.
U32 u32pack(U16 t, U16 x)
{
	return (t << 16) | (0x0000FFFF & x);
}


//convert microseconds to ticks
U16 us2tick(double t)
{
	const double aux = t * tickPerUs;
	const U16 dt_tick_MIN = 2;		//Currently, DO and AO have a latency of 2 ticks
	if ((U32)aux > 0x0000FFFF)
	{
		std::cerr << "WARNING: time step overflow. Time step set to the max: " << std::fixed << _UI16_MAX * dt_us << " us" << std::endl;
		return _UI16_MAX;
	}
	else if ((U32)aux < dt_tick_MIN)
	{
		std::cerr << "WARNING: time step underflow. Time step set to the min:" << std::fixed << dt_tick_MIN * dt_us << " us" << std::endl;;
		return dt_tick_MIN;
	}
	else
		return (U16)aux;
}


/*converts voltage (range: -10V to 10V) to a signed int 16 (range: -32768 to 32767)
0x7FFFF = 0d32767
0xFFFF = -1
0x8000 = -32768
*/
I16 volt2I16(double x)
{
	if (x > 10)
	{
		std::cerr << "WARNING: voltage overflow. Voltage set to the max: 10 V" << std::endl;
		return (I16)_I16_MAX;
	}
	else if (x < -10)
	{
		std::cerr << "WARNING: voltage underflow. Voltage set to the min: -10 V" << std::endl;
		return (I16)_I16_MIN;
	}
	else
		return (I16)(x / 10 * _I16_MAX);
}


//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't'
U32 AnalogOut(double t, double val)
{
	const U16 AOlatency_tick = 2;	//To  calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	return u32pack(us2tick(t) - AOlatency_tick, volt2I16(val));
}


//Send out a digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
U32 DigitalOut(double t, bool DO)
{
	const U16 DOlatency_tick = 2;	//To  calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
	if (DO)
		return u32pack(us2tick(t) - DOlatency_tick, 0x0001);
	else
		return u32pack(us2tick(t) - DOlatency_tick, 0x0000);
}


//Send out a pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't'
U32 PixelClock(double t, bool DO)
{
	const U16 PClatency_tick = 1;//The pixel-clock is implemented in a SCTL. I think the latency comes from reading the LUT buffer
	if (DO)
		return u32pack(us2tick(t) - PClatency_tick, 0x0001);
	else
		return u32pack(us2tick(t) - PClatency_tick, 0x0000);
}


//Push all elements of 'tailQ' into 'headQ'
U32Q PushQ(U32Q& headQ, U32Q& tailQ)
{
	while (!tailQ.empty())
	{
		headQ.push(tailQ.front());
		tailQ.pop();
	}
	return headQ;
}

int SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& QV)
{
	//take a vector of queues and return it as a single long queue
	U32Q allQs;
	for (int i = 0; i < Nchan; i++)
	{
		allQs.push(QV[i].size()); //push the number of elements in each individual queue
		while (!QV[i].empty())
		{
			allQs.push(QV[i].front());
			QV[i].pop();
		}
	}
	//transfer the queue to an array to be sent to the FPGA. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL
	const int sizeFIFOqueue = allQs.size();

	if (sizeFIFOqueue > FIFOINmax)
		std::cerr << "WARNING: FIFO IN overflow" << std::endl;

	U32* FIFO = new U32[sizeFIFOqueue];
	for (int i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQs.front();
		allQs.pop();
	}
	allQs = {};//cleanup the queue C++11 style

	//send the data to the FPGA through the FIFO
	const U32 timeout = -1; // in ms. A value -1 prevents the FIFO from timing out
	U32 r; //empty elements remaining

	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << std::endl;
	delete[] FIFO;//cleanup the array

	return 0;
}

//PARAMETERS: time step, ramp length, initial voltage, final voltage
U32Q linearRamp(double dt, double T, double Vi, double Vf)
{
	U32Q queue;
	const bool debug = false;

	if (dt < AOdt_us)
	{
		std::cerr << "WARNING: time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		dt = AOdt_us; //Analog output time increment in us
		getchar();
	}

	const int nPoints = (int)(T / dt);		//number of points

	if (nPoints <= 1)
	{
		std::cerr << "ERROR: not enought points for the linear ramp" << std::endl;
		std::cerr << "nPoints: " << nPoints << std::endl;
		getchar();
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
			const double V = Vi + (Vf - Vi)*ii / (nPoints - 1);
			queue.push(AnalogOut(dt, V));

			if (debug)
				std::cout << (ii + 1) * dt << "\t" << (ii + 1) * us2tick(dt) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
			getchar();

	}
	return queue;
}

int ReadPhotonCount(NiFpga_Status* status, NiFpga_Session session)
{
	const int ReadFifoWaitingTime = 5;			//Waiting time between each iteration
	U32 remainingFIFOa, remainingFIFOb;			//Elements remaining
	const U32 timeout = 100;					//FIFO timeout
	U32* dataFIFOa = new U32[NpixAllFrames];//The buffer size does not necessarily have to be the size of a frame

	//Initialize the array for FIFOa
	for (int ii = 0; ii < NpixAllFrames; ii++)
		dataFIFOa[ii] = 0;

	//Test for FIFOb
	//Create an array of buffer-arrays to store the data from the FIFO. The ReadFifo function gives chuncks of data.
	//Store each chunck in a separate buffer-array
	//I think I can't just make a long, concatenated 1D array because I have to pass individual arrays to the FIFO-read function
	const int NmaxbufArray = 100;
	U32** bufArrayb = new U32*[NmaxbufArray];
	for (int i = 0; i < NmaxbufArray; i++)
		bufArrayb[i] = new U32[NpixAllFrames]; //Each row is used to store the data from the ReadFifo. The buffer size could possibly be < Ntotal_pi

	//The elements in this array indicate the number of elements in each chunch of data
	int* NelementsBufArrayb = new int[NmaxbufArray];


	int NelementsReadFIFOa = 0, NelementsReadFIFOb = 0; 	//Total number of elements read from the FIFO
	int bufArrayIndexb = 0;									//Number of buffer arrays actually used
	int timeoutCounter = 100;								//Timeout the while-loop in case the data-transfer from the FIFO fails	

	//Confifure the depth of the FIFO
	//ConfigureFIFO(status, session, 1000000);

	//Declare a stopwatch
	std::clock_t start;
	double duration;

	//Start the FIFO OUT to transfer data from the FPGA FIFO to the PC FIFO
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

	//Trigger the acquisition. If triggered too early, the FPGA FIFO will probably overflow
	TriggerLineGate(status, session);

	//Start the stopwatch
	start = std::clock();

	//TODO: save the data from the FIFO saving concurrently
	//Read the PC-FIFO as the data arrive. I ran a test and found out that two 32-bit FIFOs has a larger bandwidth than a single 64 - bit FIFO
	//Test if the bandwidth can be increased by using 'NiFpga_AcquireFifoReadElementsU32'.Ref: http://zone.ni.com/reference/en-XX/help/372928H-01/capi/functions_fifo_read/
	//pass an array to a function: https://stackoverflow.com/questions/2838038/c-programming-malloc-inside-another-function
	//review of pointers and references in C++: https://www.ntu.edu.sg/home/ehchua/programming/cpp/cp4_PointerReference.html

	while (NelementsReadFIFOa < NpixAllFrames || NelementsReadFIFOb < NpixAllFrames)
	{
		//FIFO OUT a
		if (NelementsReadFIFOa < NpixAllFrames) //Skip if all the data have already been downloaded (i.e. NelementsReadFIFOa = NpixAllFrames)
		{
			//By requesting 0 elements from the FIFO, the function returns the number of elements available in the FIFO. If no data are available yet, then remainingFIFOa = 0 is returned
			NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dataFIFOa, 0, timeout, &remainingFIFOa));
			//std::cout << "Number of elements remaining in the host FIFO a: " << remainingFIFOa << std::endl;

			//If there are data available in the FIFO, retrieve it
			if (remainingFIFOa > 0)
			{
				NelementsReadFIFOa += remainingFIFOa; //Keep track of how many data points have been read so far

				//Read the elements in the FIFO
				NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dataFIFOa, remainingFIFOa, timeout, &remainingFIFOa));
				//std::cout << "aaaaaaaaaaaa: " << remainingFIFOa << std::endl;
			}
		}

		//FIFO OUT b
		if (NelementsReadFIFOb < NpixAllFrames)
		{
			NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, bufArrayb[bufArrayIndexb], 0, timeout, &remainingFIFOb));
			//std::cout << "Number of elements remaining in the host FIFO b: " << remainingFIFOb << std::endl;

			if (remainingFIFOb > 0)
			{
				NelementsReadFIFOb += remainingFIFOb;
				NelementsBufArrayb[bufArrayIndexb] = remainingFIFOb; //record how many elements are in each FIFObuffer array												

				//Read the elements in the FIFO
				NiFpga_MergeStatus(status, NiFpga_ReadFifoU32(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, bufArrayb[bufArrayIndexb], remainingFIFOb, timeout, &remainingFIFOb));
				//std::cout << "bbbbbbbbbbbbb: " << remainingFIFOb << std::endl;

				bufArrayIndexb++;

			}
		}
		Sleep(ReadFifoWaitingTime); //waiting till collecting big chuncks of data. Decrease the waiting until max transfer bandwidth

		timeoutCounter--;

		//Timeout the while loop in case the data transfer fails
		if (timeoutCounter == 0)
		{
			std::cerr << "ERROR: FIFO downloading timeout" << std::endl;
			break;
		}
	}

	//Stop the stopwatch
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Elapsed time: " << duration << " s" << std::endl;
	std::cout << "FIFO bandwidth: " << 2 * 32 * NpixAllFrames / duration / 1000000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer-arrays used: " << (U32)bufArrayIndexb << std::endl; //print how many buffer arrays were actually used
	std::cout << "Total of elements read: " << NelementsReadFIFOa << "\t" << NelementsReadFIFOb << std::endl; //print the total number of elements read


	//U32 Nfree;
	//Read the number of free spots remaining in the FIFO
	//NiFpga_MergeStatus(status, NiFpga_ReadU32(session, NiFpga_FPGAvi_IndicatorU32_FIFOOUTfreespots, &Nfree));
	//std::cout << "Number of free spots in the FIFO a: " << (U32)Nfree << std::endl;


	if (NelementsReadFIFOa == NpixAllFrames && NelementsReadFIFOb == NpixAllFrames)
	{
		unsigned char *image = UnpackFIFOBuffer(bufArrayIndexb, NelementsBufArrayb, bufArrayb);
		CorrectInterleavedImage(image);
		WriteFrameTiff(image,"_photon-counts.tif");
		//WriteFrameToTxt(image, "_photon-counts.txt");
		delete image;
	}
	else
		std::cerr << "ERROR: more or less elements received from the FIFO than expected " << std::endl;


	delete dataFIFOa;

	//clean up the buffer arrays
	for (int i = 0; i < NmaxbufArray; ++i) {
		delete[] bufArrayb[i];
	}
	delete[] bufArrayb;

	return 0;
}

//Returns a single 1D array with the chucks of data stored in the buffer 2D array
unsigned char *UnpackFIFOBuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb)
{
	const bool debug = 0; //For debugging. Generate numbers from 1 to NpixAllFrames with +1 increament

	//create a long 1D array representing the image
	static unsigned char *image = new unsigned char[NpixAllFrames];

	//initialize the array
	for (int ii = 0; ii < NpixAllFrames; ii++)
		image[ii] = 0;

	U32 pixIndex = 0;	//pixel of the image
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
int CorrectInterleavedImage(unsigned char *interleavedImage)
{
	unsigned char *auxLine = new unsigned char[Width_pixPerFrame]; //one line to temp store the data. In principle I could just use half the size, but why bothering...

	//for every odd-number line, reverse the pixel order
	for (int lineIndex = 1; lineIndex < Height_pixPerFrame; lineIndex += 2)
	{
		//save the data in an aux array
		for (int pixIndex = 0; pixIndex < Width_pixPerFrame; pixIndex++)
			auxLine[pixIndex] = interleavedImage[lineIndex*Width_pixPerFrame + (Width_pixPerFrame - pixIndex - 1)];
		//write the data back
		for (int pixIndex = 0; pixIndex < Width_pixPerFrame; pixIndex++)
			interleavedImage[lineIndex*Width_pixPerFrame + pixIndex] = auxLine[pixIndex];

	}
	delete auxLine;

	//for debugging
	//for (U32 ii = 0; ii < NpixAllFrames; ii++)
	//myfile << auxArray[ii] << std::endl;

	return 0;
}


int WriteFrameToTxt(unsigned char *imageArray, std::string fileName)
{
	//write output to txt file
	std::ofstream myfile;
	myfile.open(fileName);

	//Save the image in a text file
	for (int ii = 0; ii < NpixAllFrames; ii++)
		myfile << (int) imageArray[ii] << std::endl;

	//close txt file
	myfile.close();

	return 0;
}



/*
int i;
for(i=0; i<size; i=i+1){
data[i] = i*5000;


for(i=0; i<size; i=i+1){
printf("%i\n",data[i]);
}
getchar();
*/

/*
int16_t val = -32769;
char hex[16];
sprintf(hex, "%x", ((val + (1 << 16)) % (1 << 16)) );
puts(hex);
getchar();*/


/*the AO reads a I16, specifically
0x7FFF = 32767
0xFFFF = -1
0x8000 = -32768*/

/*
printf("%i\n", VOUT(10));
getchar();*/

/*
// Create a new queue with 'headQ' and 'tailQ' combined
U32Q NewConcatenatedQ(U32Q& headQ, U32Q& tailQ)
{
U32Q newQ;
while (!headQ.empty())
{
newQ.push(headQ.front());
headQ.pop();
}
while (!tailQ.empty())
{
newQ.push(tailQ.front());
tailQ.pop();
}
return newQ;
}
*/

//endregion "FPGA configuration"
#pragma endregion

#pragma region "FPGA initialization and trigger"

int InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));		//control-sequence trigger
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGatetrigger, 0));	//data-acquisition trigger

	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Nchannels, (U16) Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16) SyncDOtoAO_tick));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncAODOtoLineGate, (U16) SyncAODOtoLineGate_tick));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesAllFrames, (U16) NlinesAllFrames));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Height_pixPerFrame, (U16) Height_pixPerFrame));

	//Shutters
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter2, 0));
	
	//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Resonant scanner
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));	//Output voltage
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));	//Turn on/off

	//Debugging
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));	//FIFO OUT

	//Initialize all the channels with zero. No need if NiFpga_Finalize() is at the end of the main code
	/*
	U32QV QV(Nchan);
	for (U32 ii = 0; ii < Nchan; ii++)
	QV[ii].push(0);
	SendOutQueue(status, session, QV);
	TriggerFIFOIN(status, session);
	*/

	std::cout << "FPGA initialize-variables status: " << *status << std::endl;

	return 0;
}

//Main trigger. Trigger FIFO-in, which subsequently triggers AO and DO
int TriggerFIFOIN(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	std::cout << "Pulse trigger status: " << *status << std::endl;

	return 0;
}

//Trigger the 'Line gate' to start acquiring data
int TriggerLineGate(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGatetrigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGatetrigger, 0));
	std::cout << "Acquisition trigger status: " << *status << std::endl;

	return 0;
}


int ConfigureFIFO(NiFpga_Status* status, NiFpga_Session session, U32 depth)
{
	U32 actualDepth;
	NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth);
	std::cout << "actualDepth a: " << actualDepth << std::endl;
	NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth);
	std::cout << "actualDepth b: " << actualDepth << std::endl;

	return 0;
}


//endregion "FPGA configuration"
#pragma endregion

#pragma region "Vibratome"

//Start running the vibratome. Simulate the act of pushing a button on the vibratome control pad.
int Vibratome_StartStop(NiFpga_Status* status, NiFpga_Session session)
{
	const int dt = 20; //in ms. It has to be ~ 12 ms or longer to 
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 1));
	Sleep(dt);
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));

	return 0;
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
int Vibratome_SendCommand(NiFpga_Status* status, NiFpga_Session session, double dt, VibratomeChannel channel)
{
	NiFpga_FPGAvi_ControlBool selectedChannel;
	const int minstep = 10; //in ms

	switch (channel)
	{
	case VibratomeBack:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_back;
		break;
	case VibratomeForward:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_forward;
		break;
	default:
		std::cerr << "ERROR: Selected vibratome channel is unavailable" << std::endl;
		return -1;
	}


	const int delay = 1; //used to roughly calibrate the pulse length
	const int dt_ms = (int)dt / ms;

	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 1));

	if (dt_ms >= minstep)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minstep - delay);
		std::cerr << "WARNING: time step too small. Time step set to the min = ~" << minstep << "ms" << std::endl;
	}
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 0));

	return 0;
}

//endregion "Vibratome"
#pragma endregion

#pragma region "Resonant scanner"

//Start or stop the resonant scanner
NiFpga_Status ResonantScanner_StartStop(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool) state));

	return *status;
}

//Set the output voltage of the resonant scanner
int ResonantScanner_SetOutputVoltager(NiFpga_Status* status, NiFpga_Session session, double Vout)
{
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, volt2I16(Vout)));

	return 0;
}

double ResonantScanner_Amp2Volt(double Amplitude)
{
	return Amplitude * RS_voltPerUm;
}


//endregion "Resonant scanner"
#pragma endregion



#pragma region "Shutters"

int Shutter1_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter1, (NiFpga_Bool)state));

	return 0;
}

int Shutter2_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter2, (NiFpga_Bool)state));

	return 0;
}

//endregion "Shutters"
#pragma endregion


/*
//Pockels cells
NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));

*/