#include "FPGAapi.h"


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
		std::cerr << "WARNING in " << __FUNCTION__ << ": time step overflow. Time step set to the max: " << std::fixed << _UI16_MAX * dt_us << " us" << std::endl;
		return _UI16_MAX;
	}
	else if ((U32)t_tick < dt_tick_MIN)
	{
		std::cerr << "WARNING in " << __FUNCTION__ << ": time step underflow. Time step set to the min: " << std::fixed << dt_tick_MIN * dt_us << " us" << std::endl;;
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
		std::cerr << "WARNING in " << __FUNCTION__ << ": voltage overflow. Voltage set to the max: 10 V" << std::endl;
		return (I16)_I16_MAX;
	}
	else if (x < -10)
	{
		std::cerr << "WARNING in " << __FUNCTION__ << ": voltage underflow. Voltage set to the min: -10 V" << std::endl;
		return (I16)_I16_MIN;
	}
	else
		return (I16)(x / 10 * _I16_MAX);
}


//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't'
U32 singleAnalogOut(double t, double val)
{
	const U16 AOlatency_tick = 2;	//To calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	return packU32(convertUs2tick(t) - AOlatency_tick, convertVolt2I16(val));
}


//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
U32 singleDigitalOut(double t, bool DO)
{
	const U16 DOlatency_tick = 2;	//To calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
	if (DO)
		return packU32(convertUs2tick(t) - DOlatency_tick, 0x0001);
	else
		return packU32(convertUs2tick(t) - DOlatency_tick, 0x0000);
}


//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't'
U32 singlePixelClock(double t, bool DO)
{
	const U16 PClatency_tick = 1;//The pixel-clock is implemented in a SCTL. I think the latency comes from reading the LUT buffer
	if (DO)
		return packU32(convertUs2tick(t) - PClatency_tick, 0x0001);
	else
		return packU32(convertUs2tick(t) - PClatency_tick, 0x0000);
}

QU32 generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal)
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


//Push all the elements in 'tailQ' into 'headQ'
void concatenateQueues(QU32& concatenatedQueue, QU32 newQueue)
{
	while (!newQueue.empty())
	{
		concatenatedQueue.push(newQueue.front());
		newQueue.pop();
	}
}

//Send every single queue in VectorOfQueue to the FPGA bufer
//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
//Then transfer the elements in the long queue to an array to interface the FPGA
//Alternatively, the single queues could be transferred directly to the array, but why bothering...
int sendCommandsToFPGAbuffer(NiFpga_Session session, VQU32& VectorOfQueues)
{
	QU32 allQueues;								//Create a single long queue
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
		std::cerr << "WARNING in " << __FUNCTION__ << ": FIFO IN overflow" << std::endl;

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

	NiFpga_Status status = NiFpga_WriteFifoU32(session, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r);

	std::cout << "FPGA FIFO status: " << status << std::endl;
	delete[] FIFO;//cleanup the array

	return 0;
}


//endregion "FPGA low-level functions"
#pragma endregion

#pragma region "FPGA initialization and trigger"

int initializeFPGAvariables(NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_Status status;
	status = NiFpga_WriteU8(session, NiFpga_FPGAvi_ControlU8_PhotonCounterInputSelector, PhotonCounterInput);			//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
	NiFpga_MergeStatus(&status, NiFpga_WriteU8(session, NiFpga_FPGAvi_ControlU8_LineClockInputSelector, LineClockInput));					//Select the Line clock: resonant scanner or function generator
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));										//control-sequence trigger
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));									//data-acquisition trigger

	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout_tick));
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)Nchan));
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16)SyncDOtoAO_tick));
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncAODOtoLineGate, (U16)SyncAODOtoLineGate_tick));
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(NlinesAllFrames + NFrames * NlinesSkip)));			//Total number of lines in all the frames, including the skipped lines
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)HeightPerFrame_pix));								//Number of lines in a frame, without including the skipped lines
	NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(HeightPerFrame_pix + NlinesSkip)));		//Number of lines in a frame including the skipped lines

																																						//Shutters
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

	//Vibratome control
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Resonant scanner
	NiFpga_MergeStatus(&status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));											//Output voltage
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));											//Turn on/off

																																			//Debugging
	NiFpga_MergeStatus(&status, NiFpga_WriteArrayBool(session, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));										//FIFO OUT

	//Initialize all the channels with zero. No need if NiFpga_Finalize() is at the end of the main code
	/*
	U32QV vectorOfQueues(Nchan);
	for (int chan = 0; chan < Nchan; chan++)
	{
	vectorOfQueues[chan].push(0);
	}
	sendCommandsToFPGAbuffer(status, session, vectorOfQueues);
	triggerFPGAdistributeCommands(status, session);
	triggerFPGAstartImaging(status, session);
	Sleep(100);
	*/

	std::cout << "FPGA initialize-variables status: " << status << std::endl;

	return 0;
}

//Send the commands to the FPGA channel buffers but do not execute them yet
int executeFPGACommands(NiFpga_Session session)
{
	NiFpga_Status status = NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1);
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	std::cout << "Pulse trigger status: " << status << std::endl;

	return 0;
}

//Execute the commands
int triggerFPGAstartImaging(NiFpga_Session session)
{
	NiFpga_Status status = NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 1);
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));
	std::cout << "Acquisition trigger status: " << status << std::endl;

	return 0;
}

//Trigger the FIFO flushing
int triggerFIFOflush(NiFpga_Session session)
{
	NiFpga_Status status = NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1);
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
	std::cout << "Flush trigger status: " << status << std::endl;

	return 0;
}

int configureFIFO(NiFpga_Session session, U32 depth)
{
	U32 actualDepth;
	NiFpga_Status status = NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, depth, &actualDepth);
	std::cout << "actualDepth a: " << actualDepth << std::endl;
	NiFpga_MergeStatus(&status, NiFpga_ConfigureFifo2(session, NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb, depth, &actualDepth));
	std::cout << "actualDepth b: " << actualDepth << std::endl;

	return 0;
}

//endregion "FPGA initialization and trigger"
#pragma endregion


void printFPGAstatus(NiFpga_Status status, char functionName[])
{
	if (status < 0)
		std::cerr << "ERROR: '" << functionName << "' exited with FPGA code: " << status << std::endl;
	if (status >0)
		std::cerr << "WARNING: '" << functionName << "' exited with FPGA code: " << status << std::endl;
}

FPGAClassTest::FPGAClassTest()
{
	status = NiFpga_Initialize();		//Must be called before any other FPGA calls
	std::cout << "FPGA initialize status: " << status << std::endl;

	if (NiFpga_IsNotError(status))		//Check for any FPGA error
	{
		status = NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &session);		//Opens a session, downloads the bitstream
																												//1=no run, 0=run
		std::cout << "FPGA open-session status: " << status << std::endl;
	}
}

FPGAClassTest::~FPGAClassTest()
{
	if (NiFpga_IsNotError(status))
	{
		status = NiFpga_Close(session, 1);			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
																		//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
																		//0 resets, 1 does not reset
		std::cout << "FPGA closing-session status: " << status << std::endl;
	}

	//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
	status = NiFpga_Finalize();
	std::cout << "FPGA finalize status: " << status << std::endl;
};

RTsequence::RTsequence()
{
	//PixelClockEqualDuration();
	PixelClockEqualDistance();
}

RTsequence::~RTsequence()
{

}

int RTsequence::push(RTchannel chan, QU32 queue)
{
	concatenateQueues(mVectorOfQueues[chan], queue);
	return 0;
}

int RTsequence::push(RTchannel chan, U32 aa)
{
	mVectorOfQueues[chan].push(aa);
	return 0;
}


RTsequence::RTsequence(RTchannel chan, double TimeStep, double RampLength, double Vinitial, double Vfinal)
{
	mVectorOfQueues[chan] = generateLinearRamp(TimeStep, RampLength, Vinitial, Vfinal);
}

//Convert the spatial coordinate of the resonant scanner to time. x in [-RSamplitudePkPK_um/2, RSamplitudePkPK_um/2]
double RTsequence::ConvertSpatialCoord2Time(double x)
{
	const double arg = 2 * x / RSamplitudePkPK_um;
	if (arg <= 1)
		return HalfPeriodLineClock_us * asin(arg) / PI; //Return value in [-HalfPeriodLineClock_us/PI, HalfPeriodLineClock_us/PI]
	else
	{
		std::cerr << "ERROR in " << __FUNCTION__ << ": argument of asin greater than 1" << std::endl;
		return HalfPeriodLineClock_us / PI;
	}
}

//Discretize the spatial coordinate, then convert it to time
double RTsequence::getDiscreteTime(int pix)
{
	const double dx = 0.5 * um;
	return ConvertSpatialCoord2Time(dx * pix);
}

//Calculate the dwell time for the pixel
double RTsequence::calculateDwellTime(int pix)
{
	return getDiscreteTime(pix + 1) - getDiscreteTime(pix);
}

//Calculate the dwell time of the pixel but considering that the FPGA has a finite clock rate
double RTsequence::calculatePracticalDwellTime(int pix)
{
	return round(calculateDwellTime(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}

//Pixel clock sequence. Every pixel has the same duration in time.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
//Pixel clock evently spaced in time
void RTsequence::PixelClockEqualDuration()
{
	const double InitialTimeStep_us = 6.25*us;							//Relative delay of the pixel clock wrt the line clock (assuming perfect laser alignment, which is generally not true)
																		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
	mVectorOfQueues[PCLOCK].push(packU32(convertUs2tick(InitialTimeStep_us) - latency_tick, 0x0000));

	const double PixelTimeStep = 0.125 * us;
	for (int pix = 0; pix < WidthPerFrame_pix + 1; pix++)
		mVectorOfQueues[PCLOCK].push(singlePixelClock(PixelTimeStep, 1));			//Generate the pixel clock. Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter
																					//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
}

//Pixel clock sequence. Every pixel is equally spaced.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
void RTsequence::PixelClockEqualDistance()
{
	std::vector<double> PixelClockEqualDistanceLUT(WidthPerFrame_pix);

	if (WidthPerFrame_pix % 2 == 0)	//is even
	{
		for (int pix = -WidthPerFrame_pix / 2; pix < WidthPerFrame_pix / 2; pix++)	//pix in [-WidthPerFrame_pix/2,WidthPerFrame_pix/2]
			PixelClockEqualDistanceLUT[pix + WidthPerFrame_pix / 2] = calculatePracticalDwellTime(pix);
	}
	else
		std::cerr << "ERROR in " << __FUNCTION__ << ": Odd number of pixels in the image width currently not supported by the pixel clock. Pixel clock set to 0" << std::endl;

	//Determine the relative delay of the pixel clock wrt the line clock
	const U16 calibCoarse_tick = 2043;	//Look at the oscilloscope and adjust to center the pixel clock within a line scan
	const U16 calibFine_tick = 10;		//In practice, the resonant scanner is not perfectly centered around the objective's back aperture
										//Look at fluorescent beads and minimize the relative pixel shifts between forward and back scanning
	const U16 InitialTimeStep_tick = calibCoarse_tick + calibFine_tick;
	mVectorOfQueues[PCLOCK].push(packU32(InitialTimeStep_tick - latency_tick, 0x0000));

	for (int pix = 0; pix < WidthPerFrame_pix; pix++)
		mVectorOfQueues[PCLOCK].push(singlePixelClock(PixelClockEqualDistanceLUT[pix], 1));	//Generate the pixel clock.Every time HIGH is pushed, the pixel clock "ticks" (flips its requestedState), which serves as a pixel delimiter

	mVectorOfQueues[PCLOCK].push(singlePixelClock(dt_us_MIN, 1));	//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
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