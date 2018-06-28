#include "FPGAapi.h"

namespace FPGAapi
{
	//Convert time to ticks
	U16 convertUsTotick(const double t)
	{
		const double t_tick = t/us * tickPerUs;

		if ((U32)t_tick > 0x0000FFFF)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step overflow. Time step cast to the max: " << std::fixed << _UI16_MAX * usPerTick << " us" << std::endl;
			return _UI16_MAX;
		}
		else if ((U32)t_tick < tMIN_tick)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step underflow. Time step cast to the min: " << std::fixed << tMIN_tick * usPerTick << " us" << std::endl;;
			return tMIN_tick;
		}
		else
			return (U16)t_tick;
	}

	//Convert voltage to I16 [1]
	I16 convertVoltToI16(const double voltage_V)
	{
		const int VMAX = 10 * V;
		const int VMIN = -10 * V;

		if (voltage_V > 10)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage overflow. Voltage cast to the max: " + std::to_string(VMAX) + " V" << std::endl;
			return (I16)_I16_MAX;
		}
		else if (voltage_V < -10)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage underflow. Voltage cast to the min: " + std::to_string(VMIN) + " V" << std::endl;
			return (I16)_I16_MIN;
		}
		else
			return (I16)(voltage_V / 10 * _I16_MAX);
	}

	//Pack t in MSB and x in LSB. Time t and analog output AO_U16 are encoded in 16 bits each.
	U32 packU32(const U16 t_tick, const U16 AO_U16)
	{
		return (t_tick << 16) | (0x0000FFFF & AO_U16);
	}

	//Send out an analog instruction, where the analog output 'AO_V' is held for 'timeStep'
	U32 packAnalogSinglet(const double timeStep, const double AO_V)
	{
		const U16 AOlatency_tick = 2;	//To calibrate, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(convertUsTotick(timeStep) - AOlatency_tick, convertVoltToI16(AO_V));
	}

	//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 'timeStep'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
	U32 packDigitalSinglet(const double timeStep, const bool DO)
	{
		const U16 DOlatency_tick = 2;	//To calibrate, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(convertUsTotick(timeStep) - DOlatency_tick, (U16)DO);
	}

	//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 'timeStep'
	U32 packPixelclockSinglet(const double timeStep, const bool DO)
	{
		const U16 PixelclockLatency_tick = 1;//The pixel-clock is implemented using a SCTL. I think the latency comes from reading the LUT buffer
		return packU32(convertUsTotick(timeStep) - PixelclockLatency_tick, (U16)DO);
	}

	void checkStatus(char functionName[], NiFpga_Status status)
	{
		if (status < 0)
			throw  FPGAapi::FPGAexception((std::string)functionName + " with FPGA code " + std::to_string(status));
		if (status > 0)
			std::cerr << "A warning has ocurred in " << functionName << " with FPGA code " << status << std::endl;
	}

#pragma region "Session"
	Session::Session()
	{
		//Must be called before any other FPGA calls
		checkStatus(__FUNCTION__, NiFpga_Initialize());

		//Opens a session, downloads the bitstream. 1=no run, 0=run
		checkStatus(__FUNCTION__, NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &mSession));
	}

	Session::~Session()
	{
		//std::cout << "Session destructor was called" << std::endl;
	};

	void Session::initialize() const
	{
		//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
		checkStatus(__FUNCTION__, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_PhotoncounterInputSelector, photoncounterInput));				//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
		checkStatus(__FUNCTION__, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_LineclockInputSelector, lineclockInput));						//Select the Line clock: resonant scanner or function generator
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));											//Control-sequence trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));										//Data-acquisition trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_FIFOtimeout_tick, (U16)FIFOtimeout_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)nChan));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncDOtoAOtick, (U16)syncDOtoAO_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate_tick, (U16)syncAODOtoLinegate_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_Nframes, (U8)nFrames));												//Number of frames to acquire
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession,
												NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(nLinesAllFrames + nFrames * nLinesSkip - nLinesSkip)));		//Total number of lines in all the frames, including the skipped lines, minus the very last skipped lines)
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFrame,(U16)heightPerFrame_pix));							//Number of lines in a frame, without including the skipped lines
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(heightPerFrame_pix + nLinesSkip)));	//Number of lines in a frame including the skipped lines
		checkStatus(__FUNCTION__, NiFpga_WriteU32(mSession, NiFpga_FPGAvi_ControlU32_StageTriggerDuration_tick, stageTriggerDuration * tickPerUs));

		//Shutters. Commented out to allow holding the shutter on
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

		//Resonant scanner. Commented out to allow holding the RS on
		//checkStatus(__FUNCTION__,  NiFpga_WriteI16(mSession, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));	//Output voltage
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));	//Turn on/off

		//Vibratome control
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_start, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_back, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

		//Debugger
		checkStatus(__FUNCTION__, NiFpga_WriteArrayBool(mSession, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, nPulses));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_EnableFIFO, 0));															//Enable pushing data to FIFOfpga. For debugging purposes
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Pockels1_EnableAutoOff, (NiFpga_Bool)pockels1_enableAutoOff));			//Enable output gating by framegate. For debugging purposes
	
	}

	//Send every single queue in VectorOfQueue to the FPGA buffer
	//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
	//Then transfer the elements in the long queue to an array to interface the FPGA
	//Improvement: the single queues VectorOfQueues[i] could be transferred directly to the FIFO array
	void Session::writeFIFOpc(VQU32 &vectorOfQueues) const
	{
		QU32 allQueues;		//Create a single long queue
		for (int i = 0; i < nChan; i++)
		{
			allQueues.push_back(vectorOfQueues.at(i).size());		//Number of elements in the individual queue i, VectorOfQueues.at(i)

			//Non-destructive version. Randomly access the elements in VectorOfQueues[i] and push them to allQueues
			for (int iter = 0; iter < static_cast<int>(vectorOfQueues.at(i).size()); iter++)
				allQueues.push_back(vectorOfQueues.at(i).at(iter));

			/*
			//Destructive version
			while (!vectorOfQueues.at(i).empty())
			{
			allQueues.push_back(vectorOfQueues.at(i).front());	//Number of elements in the individual queue i, VectorOfQueues.at(i)
			vectorOfQueues.at(i).pop_front();
			}
			*/
		}

		const size_t sizeFIFOqueue = allQueues.size();		//Total number of elements in all the queues 

		if (sizeFIFOqueue > FIFOINmax)
			throw std::overflow_error((std::string)__FUNCTION__ + ": FIFO IN overflow");

		U32* FIFO = new U32[sizeFIFOqueue];				//Create an array for interfacing the FPGA	
		for (int i = 0; i < static_cast<int>(sizeFIFOqueue); i++)
		{
			FIFO[i] = allQueues.front();				//Transfer the queue elements to the array
			allQueues.pop_front();
		}
		allQueues = {};									//Cleanup the queue (C++11 style)

		const U32 timeout_ms = -1;		//in ms. A value -1 prevents the FIFO from timing out
		U32 r;							//Elements remaining

		checkStatus(__FUNCTION__, NiFpga_WriteFifoU32(mSession, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout_ms, &r)); //Send the data to the FPGA through the FIFO. I measure a min time of 10 ms to execute

		delete[] FIFO;					//Cleanup the array


	}

	//Execute the commands
	void Session::triggerRT() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));
	}

	void Session::enableFIFOfpga() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_EnableFIFO, 1));
	}

	//Flush the block RAMs used for buffering the pixelclock, AO, and DO 
	void Session::flushBRAMs() const
	{
		Sleep(100);	//Do not flush too soon, otherwise the output sequence will be cut off
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
		//std::cout << "flushBRAMs called\n";
	}

	//The object has to be closed explicitly in main() for now because of the exception-catching
	void Session::close(const bool reset) const
	{
		flushBRAMs();		//Flush the RAM buffers on the FPGA as precaution. Make sure that the sequence has already finished

		//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
		//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
		//0 resets, 1 does not reset
		checkStatus(__FUNCTION__, NiFpga_Close(mSession, (U32)!reset));

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		checkStatus(__FUNCTION__, NiFpga_Finalize());
	}

	NiFpga_Session Session::getSession() const
	{
		return mSession;
	}
#pragma endregion "Session"

#pragma region "RTsequence"
	RTsequence::Pixelclock::Pixelclock()
	{
		switch (pixelclockType) //pixelclockType defined globally
		{
		case uniform: pushUniformDwellTimes(15, 0.125 * us); //Dwell time = 10 * 12.5 ns = 125 ns (128 Mvps for 16X), Npix = 400
		//case uniform: pushUniformDwellTimes(15, 0.1875 * us); //Dwell time = 15 * 12.5 ns = 187.5 ns (85 Mvps for 16X), Npix = 300
			break;
		case nonuniform: pushCorrectedDwellTimes();
			break;
		default: throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pixelclock type unavailable");
			break;
		}
	}

	RTsequence::Pixelclock::~Pixelclock() {}

	//Convert the spatial coordinate of the resonant scanner to time. x in [-RSpkpk_um/2, RSpkpk_um/2]
	double RTsequence::Pixelclock::convertSpatialCoordToTime_us(const double x) const
	{
		double arg = 2 * x / RSpkpk_um;
		if (arg > 1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Argument of asin greater than 1");
		else
			return halfPeriodLineclock_us * asin(arg) / Constants::PI; //The returned value is in the range [-halfPeriodLineclock_us/PI, halfPeriodLineclock_us/PI]
	}

	//Discretize the spatial coordinate, then convert it to time
	double RTsequence::Pixelclock::getDiscreteTime_us(const int pix) const
	{
		const double dx = 0.5 * um;
		return convertSpatialCoordToTime_us(dx * pix);
	}

	//Calculate the dwell time for the pixel
	double RTsequence::Pixelclock::calculateDwellTime_us(const int pix) const
	{
		return getDiscreteTime_us(pix + 1) - getDiscreteTime_us(pix);
	}

	//Calculate the practical dwell time of each pixel, considering that the FPGA has discrete time steps
	double RTsequence::Pixelclock::calculatePracticalDwellTime_us(const int pix) const
	{
		return round(calculateDwellTime_us(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
	}

	//Pixelclock with equal dwell times
	//calibFine_tick: fine tune the pixelclock timing because of the imperfect microscope alignment
	void RTsequence::Pixelclock::pushUniformDwellTimes(const int calibFine_tick, const double dwellTime_us)
	{
		//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
		const double initialWaitingTime_us = (halfPeriodLineclock_us - widthPerFrame_pix * dwellTime_us) / 2; //Relative delay of the pixel clock wrt the line clock (assuming perfect laser alignment, which is generally not true)

		//Check if the pixelclock overflows each Lineclock
		if (initialWaitingTime_us <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");

		pixelclockQ.push_back(FPGAapi::packU32(FPGAapi::convertUsTotick(initialWaitingTime_us) + calibFine_tick - mLatency_tick, 0));	 //DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO

		//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		for (int pix = 0; pix < widthPerFrame_pix + 1; pix++)
			pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(dwellTime_us, 1));
	}

	//Pixelclock with equal pixel size (spatial).
	void RTsequence::Pixelclock::pushCorrectedDwellTimes()
	{
		//The pixel clock is triggered by the line clock (see the LV implementation) followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
		const int calibCoarse_tick = 2043;	//calibCoarse_tick: Look at the oscilloscope and adjust to center the pixel clock within a line scan
		const int calibFine_tick = 10;

		if (widthPerFrame_pix % 2 != 0)		//Throw exception if odd number of pixels (not supported yet)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Odd number of pixels for the image width currently not supported");

		//Relative delay of the pixel clock with respect to the line clock. DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO
		const U16 InitialWaitingTime_tick = (U16)(calibCoarse_tick + calibFine_tick);
		pixelclockQ.push_back(FPGAapi::packU32(InitialWaitingTime_tick - mLatency_tick, 0));

		//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
		for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)
			pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(calculatePracticalDwellTime_us(pix), 1));

		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(tMIN_us, 1));
	}

	QU32 RTsequence::Pixelclock::readPixelclock() const
	{
		return pixelclockQ;
	}


	RTsequence::RTsequence(const FPGAapi::Session &fpga) : mFpga(fpga), mVectorOfQueues(nChan)
	{
		const Pixelclock pixelclock;
		mVectorOfQueues.at(PIXELCLOCK) = pixelclock.readPixelclock();
	}

	RTsequence::~RTsequence() {}

	//Push all the elements in 'tailQ' into 'headQ'
	void RTsequence::concatenateQueues(QU32& receivingQueue, QU32& givingQueue)
	{
		while (!givingQueue.empty())
		{
			receivingQueue.push_back(givingQueue.front());
			givingQueue.pop_front();
		}
	}

	void RTsequence::pushQueue(const RTchannel chan, QU32& queue)
	{
		concatenateQueues(mVectorOfQueues.at(chan), queue);
	}

	void RTsequence::clearQueue(const RTchannel chan)
	{
		mVectorOfQueues.at(chan).clear();
	}

	void RTsequence::pushDigitalSinglet(const RTchannel chan, double timeStep, const bool DO)
	{
		mVectorOfQueues.at(chan).push_back(FPGAapi::packDigitalSinglet(timeStep, DO));
	}

	void RTsequence::pushAnalogSinglet(const RTchannel chan, double timeStep, const double AO_V)
	{
		if (timeStep < AO_tMIN_us)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_tMIN_us << " us" << std::endl;
			timeStep = AO_tMIN_us;
		}
		mVectorOfQueues.at(chan).push_back(FPGAapi::packAnalogSinglet(timeStep, AO_V));
	}

	void RTsequence::pushAnalogSingletFx2p14(const RTchannel chan, const double scalingFactor)
	{
		mVectorOfQueues.at(chan).push_back((U32)convertDoubleToFx2p14(scalingFactor));
	}

	void RTsequence::pushLinearRamp(const RTchannel chan, double timeStep, const double rampLength, const double Vi_V, const double Vf_V)
	{
		const bool debug = 0;

		if (timeStep < AO_tMIN_us)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_tMIN_us << " us" << std::endl;
			timeStep = AO_tMIN_us;		//Analog Out time increment in us
		}

		const int nPoints = (int)(rampLength / timeStep);		//Number of points

		if (nPoints <= 1)	throw std::invalid_argument((std::string)__FUNCTION__ + ": Not enought points to generate a linear ramp");

		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}

		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V = Vi_V + (Vf_V - Vi_V)*ii / (nPoints - 1);
			mVectorOfQueues.at(chan).push_back(FPGAapi::packAnalogSinglet(timeStep, V));

			if (debug)	std::cout << (ii + 1) * timeStep << "\t" << (ii + 1) * FPGAapi::convertUsTotick(timeStep) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
			getchar();

	}

	//Upload the commands to the FPGA (see the implementation of the LV code), but do not execute them yet
	void  RTsequence::uploadRT()
	{
		mFpga.writeFIFOpc(mVectorOfQueues);

		//On the FPGA, transfer the commands from FIFO IN to the sub-channel buffers
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	}

	void RTsequence::triggerRT()
	{
		mFpga.triggerRT();
	}

#pragma endregion "RTsequence"

}//namespace


/*COMMENTS

[1]
converts voltage (range: -10V to 10V) to a signed int 16 (range: -32768 to 32767)
0x7FFFF = 0d32767
0xFFFF = -1
0x8000 = -32768



*/

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

