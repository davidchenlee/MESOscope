#include "FPGAlowlevel.h"


void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

#pragma region "FPGA low-level functions"

//Pack t in MSB and x in LSB. Time t and analog output x are encoded in 16 bits each.
U32 packU32(U16 t, U16 x)
{
	return (t << 16) | (0x0000FFFF & x);
}


//Convert microseconds to ticks
U16 convertUs2tick(double t_us)
{
	const double t_tick = t_us * tickPerUs;

	if ((U32)t_tick > 0x0000FFFF)
	{
		std::cerr << "WARNING in " << __func__ << ": time step overflow. Time step set to the max: " << std::fixed << _UI16_MAX * dt_us << " us" << std::endl;
		return _UI16_MAX;
	}
	else if ((U32)t_tick < dt_tick_MIN)
	{
		std::cerr << "WARNING in " << __func__ << ": time step underflow. Time step set to the min: " << std::fixed << dt_tick_MIN * dt_us << " us" << std::endl;;
		return dt_tick_MIN;
	}
	else
		return (U16)t_tick;
}


/*converts voltage (range: -10V to 10V) to a signed int 16 (range: -32768 to 32767)
0x7FFFF = 0d32767
0xFFFF = -1
0x8000 = -32768
*/
I16 convertVolt2I16(double x)
{
	if (x > 10)
	{
		std::cerr << "WARNING in " << __func__ << ": voltage overflow. Voltage set to the max: 10 V" << std::endl;
		return (I16)_I16_MAX;
	}
	else if (x < -10)
	{
		std::cerr << "WARNING in " << __func__ << ": voltage underflow. Voltage set to the min: -10 V" << std::endl;
		return (I16)_I16_MIN;
	}
	else
		return (I16)(x / 10 * _I16_MAX);
}


//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't'
U32 generateSingleAnalogOut(double t, double val)
{
	const U16 AOlatency_tick = 2;	//To calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	return packU32(convertUs2tick(t) - AOlatency_tick, convertVolt2I16(val));
}


//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
U32 generateSingleDigitalOut(double t, bool DO)
{
	const U16 DOlatency_tick = 2;	//To calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
	if (DO)
		return packU32(convertUs2tick(t) - DOlatency_tick, 0x0001);
	else
		return packU32(convertUs2tick(t) - DOlatency_tick, 0x0000);
}


//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't'
U32 generateSinglePixelClock(double t, bool DO)
{
	const U16 PClatency_tick = 1;//The pixel-clock is implemented in a SCTL. I think the latency comes from reading the LUT buffer
	if (DO)
		return packU32(convertUs2tick(t) - PClatency_tick, 0x0001);
	else
		return packU32(convertUs2tick(t) - PClatency_tick, 0x0000);
}


//Push all the elements in 'tailQ' into 'headQ'
U32Q concatenateQueues(U32Q& headQ, U32Q& tailQ)
{
	while (!tailQ.empty())
	{
		headQ.push(tailQ.front());
		tailQ.pop();
	}
	return headQ;
}

//Send off every single queue in VectorOfQueue to the FPGA
//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
//Then transfer the elements in the long queue to an array to interface the FPGA
//Alternatively, the single queues could be transferred directly to the array, but why bothering...
int sendQueueToFPGA(NiFpga_Status* status, NiFpga_Session session, U32QV& VectorOfQueues)
{
	U32Q allQueues;								//Create a single long queue
	for (int i = 0; i < Nchan; i++)
	{
		allQueues.push(VectorOfQueues[i].size());			//Push the number of elements in each individual queue VectorOfQueues[i]
		while (!VectorOfQueues[i].empty())
		{
			allQueues.push(VectorOfQueues[i].front());		//Push all the elemets in individual queue VectorOfQueues[i] to allQueues
			VectorOfQueues[i].pop();
		}
	}

	
	const int sizeFIFOqueue = allQueues.size();		//Total number of elements in all the queues 

	if (sizeFIFOqueue > FIFOINmax)
		std::cerr << "WARNING in " << __func__ << ": FIFO IN overflow" << std::endl;

	U32* FIFO = new U32[sizeFIFOqueue];				//Create an array for interfacing the FPGA	
	for (int i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQueues.front();				//Transfer the queue elements to the array
		allQueues.pop();
	}
	allQueues = {};									//Cleanup the queue (C++11 style)

	//Send the data to the FPGA through the FIFO
	const U32 timeout = -1; // in ms. A value -1 prevents the FIFO from timing out
	U32 r; //empty elements remaining

	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << std::endl;
	delete[] FIFO;//cleanup the array

	return 0;
}

U32Q generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal)
{
	U32Q queue;
	const bool debug = 0;

	if (TimeStep < AOdt_us)
	{
		std::cerr << "WARNING in " << __func__ << ": time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		TimeStep = AOdt_us;						//Analog output time increment (in us)
		return {};
	}

	const int nPoints = (int)(RampLength / TimeStep);		//Number of points

	if (nPoints <= 1)
	{
		std::cerr << "ERROR in " << __func__ << ": not enought points for the linear ramp" << std::endl;
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
			queue.push(generateSingleAnalogOut(TimeStep, V));

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

int readPhotonCount(NiFpga_Status* status, NiFpga_Session session)
{
	//FIFOa
	int NelementsReadFIFOa = 0; 						//Total number of elements read from the FIFO
	U32 *dataFIFOa = new U32[NpixAllFrames];			//The buffer size does not necessarily have to be the size of a frame

	//Initialize the array for FIFOa
	for (int ii = 0; ii < NpixAllFrames; ii++)
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
	triggerFPGAstartsImaging(status, session);

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
					throw std::range_error(std::string{} + "ERROR in " + __func__ + ": Buffer array overflow\n");
				}

				bufArrayIndexb++;
			}
		}

		timeoutCounter--;

		//Timeout the while loop in case the data transfer fails
		if (timeoutCounter == 0)
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
		myfile << (int) imageArray[ii] << std::endl;	//Write each element

	myfile.close();										//Close the txt file

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


//endregion "FPGA configuration"
#pragma endregion

#pragma region "FPGA trigger"

//Main trigger. Trigger FIFO-in, which subsequently triggers AO and DO
int triggerFPGAreadsCommandsFromPC(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	std::cout << "Pulse trigger status: " << *status << std::endl;

	return 0;
}

//Trigger the 'Line gate' to start acquiring data
int triggerFPGAstartsImaging(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));
	std::cout << "Acquisition trigger status: " << *status << std::endl;

	return 0;
}

//Trigger the 'Line gate' to start acquiring data
int triggerFIFOflush(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
	std::cout << "Flush trigger status: " << *status << std::endl;

	return 0;
}

int configureFIFO(NiFpga_Status* status, NiFpga_Session session, U32 depth)
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
int vibratome_StartStop(NiFpga_Status* status, NiFpga_Session session)
{
	const int WaitingTime = 20; //in ms. It has to be ~ 12 ms or longer to 
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 1));
	Sleep(WaitingTime);
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));

	return 0;
}

//Simulate the act of pushing a button on the vibratome control pad. The timing fluctuates approx in 1ms
int vibratome_SendCommand(NiFpga_Status* status, NiFpga_Session session, double pushDuration, VibratomeChannel channel)
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


	const int delay = 1;						//Used to roughly calibrate the pulse length
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

//endregion "Vibratome"
#pragma endregion

#pragma region "Resonant scanner"

//Start or stop the resonant scanner
NiFpga_Status resonantScanner_StartStop(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, (NiFpga_Bool) state));

	return *status;
}

//Set the output voltage of the resonant scanner
int resonantScanner_SetOutputVoltager(NiFpga_Status* status, NiFpga_Session session, double Vout)
{
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, convertVolt2I16(Vout)));

	return 0;
}

double resonantScanner_Amp2Volt(double Amplitude)
{
	return Amplitude * RS_voltPerUm;
}


//endregion "Resonant scanner"
#pragma endregion

#pragma region "Shutters"

int shutter1_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter1, (NiFpga_Bool)state));

	return 0;
}

int shutter2_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter2, (NiFpga_Bool)state));

	return 0;
}

//endregion "Shutters"
#pragma endregion

#pragma region "Pockels cells"
//Pockels cells
//NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));

//endregion "Pockels cells"
#pragma endregion