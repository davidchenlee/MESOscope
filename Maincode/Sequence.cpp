#include "Sequence.h"

#pragma region "Low-level functions"

void printHex(U32 input)
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
	double aux = t * tickPerUs;
	U16 dt_tick_MIN = 2;		//Currently, DO and AO have a latency of 2 ticks
	if ((U32)aux > 0x0000FFFF)
	{
		std::cout << "WARNING: time step overflow. Time step set to the max: " << std::fixed << _UI16_MAX * dt_us << " us" << std::endl;
		return _UI16_MAX;
	}
	else if ((U32)aux < dt_tick_MIN)
	{
		std::cout << "WARNING: time step underflow. Time step set to the min:" << std::fixed << dt_tick_MIN * dt_us << " us" << std::endl;;
		return dt_tick_MIN;
	}
	else
		return (U16)aux;
}


//converts voltage (range: -10V to 10V) to a signed int 16 (range: -32768 to 32767)
//0x7FFFF = 0d32767
//0xFFFF = -1
//0x8000 = -32768
I16 volt2I16(double x)
{
	if (x > 10)
	{
		std::cout << "WARNING: voltage overflow. Voltage set to the max: 10 V" << std::endl;
		return (U16)_I16_MAX;
	}
	else if (x < -10)
	{
		std::cout << "WARNING: voltage underflow. Voltage set to the min: -10 V" << std::endl;
		return (U16)_I16_MIN;
	}
	else
		return (U16)(x / 10 * _I16_MAX);
}


//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't'
U32 AnalogOut(double t, double val)
{
	U16 AOlatency_tick = 2;	//To  calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	return u32pack(us2tick(t) - AOlatency_tick, volt2I16(val));
}


//Send out a digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
U32 DigitalOut(double t, bool DO)
{
	U16 DOlatency_tick = 2;	//To  calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
	if (DO)
		return u32pack(us2tick(t) - DOlatency_tick, 0x0001);
	else
		return u32pack(us2tick(t) - DOlatency_tick, 0x0000);
}


//Send out a pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't'
U32 PixelClock(double t, bool DO)
{
	U16 PClatency_tick = 1;//The pixel-clock is implemented in a SCTL. I think the latency comes from reading the LUT buffer
	if (DO)
		return u32pack(us2tick(t) - PClatency_tick, 0x0001);
	else
		return u32pack(us2tick(t) - PClatency_tick, 0x0000);
}


// Push all elements of 'tailQ' into 'headQ'
U32Q PushQ(U32Q& headQ, U32Q& tailQ)
{
	while (!tailQ.empty())
	{
		headQ.push(tailQ.front());
		tailQ.pop();
	}
	return headQ;
}


// PARAMETERS: time step, ramp length, initial voltage, final voltage
U32Q linearRamp(double dt, double T, double Vi, double Vf)
{
	U32Q queue;
	bool debug = false;

	if (dt < AOdt_us)
	{
		std::cout << "WARNING: time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		dt = AOdt_us; //Analog output time increment in us
		getchar();
	}

	U32 nPoints = (U32)(T / dt);		//number of points

	if (nPoints <= 1)
	{
		std::cout << "ERROR: not enought points for the linear ramp" << std::endl;
		std::cout << "nPoints: " << nPoints << std::endl;
		getchar();
	}
	else
	{
		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}

		for (U32 ii = 0; ii < nPoints; ii++)
		{
			double V = Vi + (Vf - Vi)*ii / (nPoints - 1);
			queue.push(AnalogOut(dt, V));

			if (debug)
				std::cout << (ii + 1) * dt << "\t" << (ii + 1) * us2tick(dt) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
			getchar();

	}
	return queue;
}


void CountPhotons(NiFpga_Status* status, NiFpga_Session session)
{

	//write output to txt file
	std::ofstream myfile;
	myfile.open("_photon-counts.txt");


	U32 ReadFifoWaittime = 1;			//Wait time between each iteration
	U32 remainingFIFOa, remainingFIFOb; //Elements remaining
	U32 timeout = 100;
	U32* dataFIFOa = new U32[Ntotal_pix];

	//Initialize the array for FIFOa
	for (U32 ii = 0; ii < Ntotal_pix; ii++)
		dataFIFOa[ii] = 0;

	//Test for FIFOb
	//Create an array of buffer-arrays to store the data from the FIFO. The ReadFifo function gives chuncks of data.
	//Store each chunck in a separate buffer-array
	//I think I can't just make a long, concatenated 1D array because I have to pass individual arrays to the FIFO-read function
	U32 NmaxbufArray = 10;
	U32** bufArrayb = new U32*[NmaxbufArray];
	for (U32 i = 0; i < NmaxbufArray; i++)
		bufArrayb[i] = new U32[Ntotal_pix]; //Each row is used to store the data from the ReadFifo

	//The elements in this array indicate the number of elements in each chunch of data
	U32* NelementsBufArrayb = new U32[NmaxbufArray];

	
	//U32 actualDepth;
	//NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, 1000000, &actualDepth);
	//std::cout << "actualDepth a: " << actualDepth << std::endl;
	//NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, 1000000, &actualDepth);
	//std::cout << "actualDepth b: " << actualDepth << std::endl;


	U32 NelementsReadFIFOa = 0, NelementsReadFIFOb = 0; 	//Total number of elements read from the FIFO
	U32 bufArrayIndexb = 0;									//Number of buffer arrays actually used
    U32 timeoutCounter = 100;								//Timeout the while-loop in case the data transfer from the FIFO fails	
													
	std::clock_t start;										//Declare a stopwatch
	double duration;

	//Start transfering the data in the FPGA FIFO to the PC FIFO
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb));

	//Trigger the acquisition. If triggered too early, the FPGA FIFO will probably overflow
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Start_acquisition, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Start_acquisition, 0));



	start = std::clock(); //Start the timer

	//TODO: implement the FIFO reading and data saving concurrently
	//Read the PC-FIFO as the data arrive. I ran a test and found out that two 32-bit FIFOs has a larger bandwidth than a single 64 - bit FIFO
	while (NelementsReadFIFOa < Ntotal_pix || NelementsReadFIFOb < Ntotal_pix)
	{
		//FIFO OUT a
		if (NelementsReadFIFOa < Ntotal_pix) //Skip if all the data have already been downloaded (i.e. NelementsReadFIFOa = Ntotal_pix)
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
		if (NelementsReadFIFOb < Ntotal_pix)
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
		Sleep(ReadFifoWaittime); //wait till collecting big chuncks of data. Decrease the wait until max transfer bandwidth

		timeoutCounter--;

		//Timeout the while loop in case the data transfer fails
		if (timeoutCounter == 0)
		{
			std::cout << "ERROR: FIFO downloading timeout" << std::endl;
			break;
		}	
	}
	
	//Stop the timer
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	std::cout << "Elapsed time: " << duration << " s" << std::endl;
	std::cout << "FIFO bandwidth: " << 2 * 32 * Ntotal_pix / duration / 1000000 << " Mbps" << std::endl; //2 FIFOs of 32 bits each

	std::cout << "Buffer-arrays used: " << bufArrayIndexb << std::endl; //print how many buffer arrays were actually used
	std::cout << "Total of elements read: " << NelementsReadFIFOa << "\t" << NelementsReadFIFOb << std::endl; //print the total number of elements read


	//U32 Nfree;
	//Read the number of free spots remaining in the FIFO
	//NiFpga_MergeStatus(status, NiFpga_ReadU32(session, NiFpga_FPGAvi_IndicatorU32_FIFOOUTfreespots, &Nfree));
	//std::cout << "Number of free spots in the FIFO a: " << (U32)Nfree << std::endl;
	

	if (NelementsReadFIFOa == Ntotal_pix || NelementsReadFIFOb == Ntotal_pix)
	{
		//create a long 1D array representing the image
		U32 *image = new U32[Width_pix*Height_pix];

		//initialize the array
		for (U32 ii = 0; ii < Width_pix*Height_pix; ii++)
			image[ii] = 0;

		U32 index = 0;
		for (U32 ii = 0; ii < bufArrayIndexb; ii++)
		{
			for (U32 jj = 0; jj < NelementsBufArrayb[ii]; jj++)
			{
				myfile << bufArrayb[ii][jj] << std::endl;		//Save the buffer-arrays into a text file
				image[index] = bufArrayb[ii][jj];
				//image[index] = index+1;						//for debugging
				index++;
			}
		}

		//Reverse the pixel order every other line. I'm just gonna use an aux array for now.
		//Later on, write the tiff directly from the buffer arrays. to deal with segmented pointers, use memcpy, memset, memmove or the Tiff versions for such functions
		//memset http://www.cplusplus.com/reference/cstring/memset/
		//memmove http://www.cplusplus.com/reference/cstring/memmove/
		//One idea is to read bufArrayb line by line (1 line = Width_pix x 1) and save it to file using TIFFWriteScanline
		U32 *auxArray = new U32[Width_pix*Height_pix];
		//initialize the array
		for (U32 ii = 0; ii < Width_pix*Height_pix; ii++)
			auxArray[ii] = 0;

		for (U32 lineIndex = 0; lineIndex < Height_pix; lineIndex++)
			for (U32 pixIndex = 0; pixIndex < Width_pix; pixIndex++)
			{
				if (lineIndex % 2)
					auxArray[lineIndex*Width_pix + pixIndex] = image[lineIndex*Width_pix + (Width_pix - pixIndex - 1)]; //reversed case
				else
					auxArray[lineIndex*Width_pix + pixIndex] = image[lineIndex*Width_pix + pixIndex];
			}


		//for debugging
		//for (U32 ii = 0; ii < Width_pix*Height_pix; ii++)
		//myfile << auxArray[ii] << std::endl;

		delete image, auxArray;
	}
	else
		std::cout << "ERROR: some elements in the FIFO are missing " << std::endl;

	
	//close txt file
	myfile.close();

	
	delete dataFIFOa;
	
	//clean up the buffer arrays
	for (U32 i = 0; i < NmaxbufArray; ++i) {
		delete[] bufArrayb[i];
	}
	delete[] bufArrayb;

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

//endregion "Low-level functions"
#pragma endregion

#pragma region "Individual sequences"

//Linearly scan the galvo while the RS is on to acquire a 2D image
U32QV Acquire2D()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA

					 //Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	//linear ramp for the galvo
	double Vmax = 1.5;
	double step = 8 * us;


	U32Q linearRampQueue; //Create a queue for the ramps
	U32Q linearRampSegment0 = linearRamp(step, 25 * ms, -Vmax, Vmax); //ramp up the galvo from -Vmax to Vmax
	U32Q linearRampSegment1 = linearRamp(step, 5 * ms, Vmax, -Vmax);  //set the galvo back to -Vmax
	PushQ(linearRampQueue, linearRampSegment0);
	PushQ(linearRampQueue, linearRampSegment1);

	//AO0 = AO1. TRIGGERED BY CONN1/DIO16
	QV[ABUF0] = linearRampQueue;

	//DO0
	QV[DBUF0].push(DigitalOut(4 * us, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));


	return QV;
}


//Test the analog and digital output
U32QV TestAODOSeq()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA

					 //AO0
	QV[ABUF0].push(AnalogOut(4 * us, 10));
	QV[ABUF0].push(AnalogOut(4 * us, 0));
	QV[ABUF0].push(AnalogOut(4 * us, 10));
	QV[ABUF0].push(AnalogOut(4 * us, 0));//go back to zero

										 //DO0
	QV[DBUF0].push(DigitalOut(4 * us, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));

	//QV[AO0] = GalvoSeq();


	//CURRENTLY, AO1 AND DO1 ARE TRIGGERED BY CONN1/DIO16
	//AO1
	QV[ABUF1].push(AnalogOut(4 * us, 5));
	QV[ABUF1].push(AnalogOut(4 * us, 0));
	QV[ABUF1].push(AnalogOut(4 * us, 5));
	QV[ABUF1].push(AnalogOut(4 * us, 0));

	//DO1
	QV[DBUF1].push(DigitalOut(4 * us, 1));
	QV[DBUF1].push(DigitalOut(4 * us, 0));
	QV[DBUF1].push(DigitalOut(4 * us, 0));
	QV[DBUF1].push(DigitalOut(4 * us, 0));

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	return QV;
}


//Pixel clock sequence. The pixel clock starts when the line clock ticks, followed by a wait time 't'
//At 160MHz, the clock increment is 6.25ns = 0.00625us
U32Q PixelClockSeq()
{
	U32Q Q;	//Create a queue

			//INITIAL WAIT TIME. Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us, then the wait time is (62.5-50)/2 = 6.25us
	double t = 6.25*us;
	U16 latency = 2; //latency of detecting the line clock. Calibrate the latency on the oscilloscope
	Q.push(u32pack(us2tick(t) - latency, 0x0000));

	//PIXEL CLOCK TICKS. Everytime HIGH is pushed, the pixel clock "ticks" (flips its state)
	for (U16 ii = 0; ii < Width_pix + 1; ii++) // Npixels+1 because there is one more pixel-clock tick than number of pixels
		Q.push(PixelClock(0.125 * us, 1));
	//Q.push(PixelClock(0.0625 * us, 1));
	return Q; //this returns a queue and not a vector of queues
}



U32Q GalvoLinearRamp()
{
	double Vmax = 5;
	double step = 4 * us;
	U32Q Q; //Create a queue

			//linear output
	U32Q linearRamp1 = linearRamp(step, 1 * ms, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 25 * ms, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 1 * ms, Vmax, 0);
	//PushQ(Q, linearRamp1);
	PushQ(Q, linearRamp2);
	//PushQ(Q, linearRamp3);
	return Q; //this returns a queue and not a vector of queues
}


U32QV GalvoTest()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	QV[ABUF0] = GalvoLinearRamp();

	double pulsewidth = 300 * us;

	/*
	QV[AO0].push(AnalogOut(4 * us, 0.000));
	QV[AO0].push(AnalogOut(pulsewidth, 5));
	QV[AO0].push(AnalogOut(4 * us, 0.000));
	*/

	QV[DBUF0].push(DigitalOut(pulsewidth, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	return QV;
}


U32QV DigitalTimingCheck()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double step = 400 * us;

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}


U32QV DigitalLatencyCalib()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double step = 4 * us;

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));

	for (U32 ii = 0; ii < 99; ii++)
		QV[DBUF0].push(DigitalOut(step, 0));

	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}


//Calibrate the digital channels first, then use it as a time reference
U32QV AnalogLatencyCalib()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double delay = 400 * us;
	double step = 4 * us;

	//AO0
	QV[ABUF0].push(AnalogOut(step, 10));//initial pulse
	QV[ABUF0].push(AnalogOut(step, 0));
	QV[ABUF0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[ABUF0].push(AnalogOut(step, 5));//final pulse
	QV[ABUF0].push(AnalogOut(step, 0));

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));
	QV[DBUF0].push(DigitalOut(delay, 0));
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}


//Start running the vibratome
int StartVT(NiFpga_Status* status, NiFpga_Session session)
{
	int dt = 20; //in ms. It has to be ~ 12 ms or longer to 
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 1));
	Sleep(dt);
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));

	return 0;
}

//Non-deterministic digital pulse for the vibratome control. The timing fluctuates approx in 1ms
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel)
{
	U8 selectedChannel;
	int minstep = 10; //in ms

	switch (channel)
	{
	case VTback:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_back;
		break;
	case VTforward:
		selectedChannel = NiFpga_FPGAvi_ControlBool_VT_forward;
		break;
	default:
		std::cout << "ERROR: Selected VT channel is unavailable" << std::endl;
		return -1;
	}


	int delay = 1; //used to roughly calibrate the pulse length
	int dt_ms = (int)dt / ms;

	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 1));

	if (dt_ms >= minstep)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minstep - delay);
		std::cout << "WARNING: time step too small. Time step set to the min = ~" << minstep << "ms" << std::endl;
	}
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 0));

	return 0;
}

//endregion "Individual sequences"
#pragma endregion

#pragma region "FPGA functions"

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Trigger, 0));//control-sequence trigger
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Start_acquisition, 0)); //data-acquisition trigger

	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_FIFO_timeout, FIFOtimeout));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Nchannels, Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Sync_DO_to_AO, Sync_DO_to_AO));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Sync_AODO_to_LineGate, Sync_AODO_to_LineGate));
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Height_pix, Height_pix));

	//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Debug FIFO OUT
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));

	//Initialize all the channels with zero. Not needed if NiFpga_Finalize() is at the end of the main code
	/*
	U32QV QV(Nchan);
	for (U8 ii = 0; ii < Nchan; ii++)
	QV[ii].push(0);
	SendOutQueue(status, session, QV);
	TriggerAODO(status, session);
	*/

	std::cout << "FPGA initialize-variables status: " << *status << std::endl;
}



void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& QV)
{

	//take a vector of queues and return it as a single queue
	U32Q allQs;
	for (U32 i = 0; i < Nchan; i++)
	{
		allQs.push(QV[i].size()); //push the number of elements in the single queue
		while (!QV[i].empty())
		{
			allQs.push(QV[i].front());
			QV[i].pop();
		}
	}
	//transfer the queue to an array to be sent to the FPGA. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL
	U32 sizeFIFOqueue = allQs.size();

	if (sizeFIFOqueue > FIFOINmax)
		std::cout << "WARNING: FIFO IN overflow" << std::endl;

	U32* FIFO = new U32[sizeFIFOqueue];
	for (U32 i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQs.front();
		allQs.pop();
	}
	allQs = {};//cleanup the queue in C++11

			   //send the data to the FPGA through the FIFO
	U32 timeout = -1; // in ms. A value -1 prevents the FIFO from timing out
	U32 r; //empty elements remaining

	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << std::endl;
	delete[] FIFO;//cleanup the array
}

//Main trigger. Trigger FIFO-in, which subsequently triggers AO and DO
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Trigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Trigger, 0));
	std::cout << "Pulse trigger status: " << *status << std::endl;
}

//Trigger the pixel clock, and therefore, counters, and FIFO-out
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Start_acquisition, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Start_acquisition, 0));
	std::cout << "Acquisition trigger status: " << *status << std::endl;
}

//endregion "FPGA functions"
#pragma endregion

#pragma region "Combined sequences"

void MainSequence(NiFpga_Status* status, NiFpga_Session session)
{
	//control sequences
	//SendOutQueue(status, session, GalvoTest());
	SendOutQueue(status, session, Acquire2D());
	TriggerAODO(status, session);			//trigger the analog and digital outputs

											//TriggerAcquisition(status, session); // trigger the data acquisition
	CountPhotons(status, session);


	//SECOND ROUND
	if (0)
	{
		SendOutQueue(status, session, TestAODOSeq());
		TriggerAODO(status, session);
		TriggerAcquisition(status, session);
	}
}

/* Test the Seq class
void SeqClassTest()
{
Seq ss;
ss.shutter(1 * us, 1);
std::cout << "size of the vector" << ss.size() << std::endl;
std::cout << "" << (ss.vector())[0].size() << std::endl;
Sleep(1000);
}
*/

//endregion "FPGA functions"
#pragma endregion