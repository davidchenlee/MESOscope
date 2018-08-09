#include "FPGAapi.h"

namespace FPGAns
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

	//Convert voltage (-10V to 10V) to I16 (-32768 to 32767)
	//Examples of I16 numbers:
	//0x7FFF = 32767
	//0xFFFF = -1
	//0x8000 = -32768
	I16 convertVoltToI16(const double voltage_V)
	{
		if (voltage_V > AOmax_V)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage overflow. Voltage cast to the max: " + std::to_string(AOmax_V) + " V" << std::endl;
			return (I16)_I16_MAX;
		}
		else if (voltage_V < -AOmax_V)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage underflow. Voltage cast to the min: " + std::to_string(-AOmax_V) + " V" << std::endl;
			return (I16)_I16_MIN;
		}
		else
			return (I16)(voltage_V / AOmax_V * _I16_MAX);
	}

	//Convert I16 (-32768 to 32767) to voltage (-10V to 10V)
	double convertI16toVolt(const int input)
	{
		//Positive case
		if (input >= 0)
		{
			//Check for overflow
			if (input > _I16_MAX)
			{
				std::cerr << "WARNING in " << __FUNCTION__ << ": Input overflow, _I16_MAX used instead" << std::endl;
				return AOmax_V;
			}
			else
				return 1.0 * input / _I16_MAX * AOmax_V;
		}
		else //Negative case
		{
			//Check for underoverflow
			if (input < _I16_MIN)
			{
				std::cerr << "WARNING in " << __FUNCTION__ << ": Input underflow, _I16_MIN used instead" << std::endl;
				return -AOmax_V;
			}
			else
				return -1.0 * input / _I16_MIN * AOmax_V;
		}		
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
			throw  FPGAns::FPGAexception((std::string)functionName + " with FPGA code " + std::to_string(status));
		if (status > 0)
			std::cerr << "A warning has ocurred in " << functionName << " with FPGA code " << status << std::endl;
	}

#pragma region "FPGA"
	FPGA::FPGA()
	{
		//Must be called before any other FPGA calls
		checkStatus(__FUNCTION__, NiFpga_Initialize());

		//Opens a session, uploads the bitfile to the FPGA. 1=no run, 0=run
		checkStatus(__FUNCTION__, NiFpga_Open(mBitfile.c_str(), NiFpga_FPGAvi_Signature, "RIO0", 0, &mFpgaHandle));

		//Set up the FPGA parameters
		initializeFpga_();
	}

	FPGA::~FPGA()
	{
		//std::cout << "FPGA destructor was called" << std::endl;
	};

	//The object has to be closed explicitly because of the exception catching
	void FPGA::close(const bool reset) const
	{
		//Flush the RAM buffers on the FPGA as precaution. Make sure that the sequence has already finished
		Sleep(100);	//Do not flush too soon, otherwise the output sequence will be cut off
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpgaHandle, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpgaHandle, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
		//std::cout << "flushBRAMs called\n";

		//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
		//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
		//0 resets, 1 does not reset
		checkStatus(__FUNCTION__, NiFpga_Close(mFpgaHandle, (U32)!reset));

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		checkStatus(__FUNCTION__, NiFpga_Finalize());
	}

	NiFpga_Session FPGA::getFpgaHandle() const
	{
		return mFpgaHandle;
	}

	//Load the imaging parameters onto the FPGA. See 'Const.cpp' for the definition of each variable
	void FPGA::initializeFpga_() const
	{
		if (nChan < 0 || FIFOINtimeout_tick < 0 || syncDOtoAO_tick < 0 || syncAODOtoLinegate_tick < 0 || linegateTimeout_us < 0 || stageTriggerPulse_ms < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": One or more imaging parameters take negative values");

		//INPUT SELECTORS
		checkStatus(__FUNCTION__, NiFpga_WriteU8(getFpgaHandle(), NiFpga_FPGAvi_ControlU8_PhotoncounterInputSelector, photoncounterInput));						//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
		checkStatus(__FUNCTION__, NiFpga_WriteArrayBool(getFpgaHandle(), NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, nPulses));					//For debugging the photoncounters

		//FIFOIN
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getFpgaHandle(), NiFpga_FPGAvi_ControlU16_Nchannels, (U16)nChan));											//Number of input channels
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));												//Control-sequence trigger
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getFpgaHandle(), NiFpga_FPGAvi_ControlU16_FIFOINtimeout_tick, (U16)FIFOINtimeout_tick));						//FIFOIN timeout

		//FIFOOUT
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTfpgaEnable, FIFOOUTfpgaEnable));							//Enable pushing data to FIFOOUTfpga. For debugging purposes

		//TRIGGERS AND DELAYS
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));												//Data-acquisition trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));												//Memory-flush trigger
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getFpgaHandle(), NiFpga_FPGAvi_ControlU16_SyncDOtoAOtick, (U16)syncDOtoAO_tick));								//DO and AO relative sync
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getFpgaHandle(), NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate_tick, (U16)syncAODOtoLinegate_tick));			//DO and AO sync to linegate

		if (linegateTimeout_us <= 2 * halfPeriodLineclock_us)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The linegate timeout must be greater than the lineclock period");
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getFpgaHandle(), NiFpga_FPGAvi_ControlU16_LinegateTimeout_tick, (U16)(linegateTimeout_us * tickPerUs)));		//Sequence trigger timeout

		//POCKELS CELLS
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_Pockels1_EnableAutoOff, (NiFpga_Bool)pockels1_enableAutoOff));	//Enable gating the pockels by framegate. For debugging purposes

		//VIBRATOME
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTstart, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTback, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_VTforward, 0));

		//STAGES
		const bool scanDirection = 0;
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getFpgaHandle(), NiFpga_FPGAvi_ControlU32_StageTriggerPulse_tick, (U32)stageTriggerPulse_ms * tickPerUs));	//Trigger pulse width
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getFpgaHandle(), NiFpga_FPGAvi_ControlBool_ScanDirection, scanDirection));									//Z-stage scan direction (1 for up, 0 for down)

		/*
		//SHUTTERS. Commented out to allow keeping the shutter on
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_Shutter1, 0));
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_Shutter2, 0));

		//RESONANT SCANNER. Commented out to allow keeping the RS on
		checkStatus(__FUNCTION__,  NiFpga_WriteI16(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlI16_RScontrol_I16, 0));	//Output voltage
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_RSenable, 0));	//Turn on/off
		*/
	}

#pragma endregion "FPGA"

#pragma region "RTsequence"
	RTsequence::Pixelclock::Pixelclock(const int widthPerFrame_pix, const double dwell_us): mWidthPerFrame_pix(widthPerFrame_pix), mDwell_us(dwell_us)
	{
		switch (pixelclockType)
		{
		case uniform: pushUniformDwellTimes(-32);
			break;
		//case nonuniform: pushCorrectedDwellTimes();
			//break;
		default: throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pixelclock type unavailable");
			break;
		}
	}

	RTsequence::Pixelclock::~Pixelclock() {}


	//Pixelclock with equal dwell times
	//calibFine_tick: fine tune the pixelclock timing
	void RTsequence::Pixelclock::pushUniformDwellTimes(const int calibFine_tick)
	{
		//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
		//For example, for a dwell time = 125ns and 400 pixels, the initial waiting time is (halfPeriodLineclock_us-400*125ns)/2

		const double initialWaitingTime_us = (halfPeriodLineclock_us - mWidthPerFrame_pix * mDwell_us) / 2; //Relative delay of the pixel clock wrt the line clock

		//Check if the pixelclock overflows each Lineclock
		if (initialWaitingTime_us <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");

		mPixelclockQ.push_back(FPGAns::packU32(FPGAns::convertUsTotick(initialWaitingTime_us) + calibFine_tick - mLatency_tick, 0));	 //DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO

		//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		for (int pix = 0; pix < mWidthPerFrame_pix + 1; pix++)
			mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(mDwell_us, 1));
	}

	QU32 RTsequence::Pixelclock::readPixelclock() const
	{
		return mPixelclockQ;
	}

	RTsequence::RTsequence(const FPGAns::FPGA &fpga, const LineclockInputSelector lineclockInput, const int nFrames, const int widthPerFrame_pix, const int heightPerFrame_pix) :
		mFpga(fpga), mVectorOfQueues(nChan), mLineclockInput(lineclockInput), mNframes(nFrames), mWidthPerFrame_pix(widthPerFrame_pix), mHeightPerFrame_pix(heightPerFrame_pix)
	{
		//Set the imaging parameters
		mHeightAllFrames_pix = mHeightPerFrame_pix * mNframes;
		mNpixAllFrames = mWidthPerFrame_pix * mHeightAllFrames_pix;
		uploadImagingParameters_();

		//Generate a pixelclock
		const Pixelclock pixelclock(mWidthPerFrame_pix, mDwell_us);
		mVectorOfQueues.at(PIXELCLOCK) = pixelclock.readPixelclock();
	}

	//Load the imaging parameters onto the FPGA
	void RTsequence::uploadImagingParameters_() const
	{
		if (mNframes < 0 || mHeightAllFrames_pix < 0 || mNlinesSkip < 0 || mHeightPerFrame_pix < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": One or more imaging parameters take negative values");

		checkStatus(__FUNCTION__, NiFpga_WriteU8(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlU8_Nframes, (U8)mNframes));												//Number of frames to acquire
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mFpga.getFpgaHandle(),
			NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(mHeightAllFrames_pix + mNframes * mNlinesSkip - mNlinesSkip)));													//Total number of lines in all the frames, including the skipped lines, minus the very last skipped lines)
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)mHeightPerFrame_pix));							//Number of lines in a frame, without including the skipped lines
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(mHeightPerFrame_pix + mNlinesSkip)));	//Number of lines in a frame including the skipped lines
	
		//FG, RS selector
		checkStatus(__FUNCTION__, NiFpga_WriteU8(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlU8_LineclockInputSelector, mLineclockInput));							//Select the Line clock: resonant scanner or function generator
	}

	RTsequence::~RTsequence() {}

	//Push all the elements in 'tailQ' into 'headQ'
	void RTsequence::concatenateQueues_(QU32& receivingQueue, QU32& givingQueue) const
	{
		while (!givingQueue.empty())
		{
			receivingQueue.push_back(givingQueue.front());
			givingQueue.pop_front();
		}
	}

	void RTsequence::pushQueue(const RTchannel chan, QU32& queue)
	{
		concatenateQueues_(mVectorOfQueues.at(chan), queue);
	}

	void RTsequence::clearQueue(const RTchannel chan)
	{
		mVectorOfQueues.at(chan).clear();
	}

	void RTsequence::pushDigitalSinglet(const RTchannel chan, double timeStep, const bool DO)
	{
		mVectorOfQueues.at(chan).push_back(FPGAns::packDigitalSinglet(timeStep, DO));
	}

	void RTsequence::pushAnalogSinglet(const RTchannel chan, double timeStep, const double AO_V)
	{
		if (timeStep < AO_tMIN_us)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_tMIN_us << " us" << std::endl;
			timeStep = AO_tMIN_us;
		}
		mVectorOfQueues.at(chan).push_back(FPGAns::packAnalogSinglet(timeStep, AO_V));
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
			mVectorOfQueues.at(chan).push_back(FPGAns::packAnalogSinglet(timeStep, V));

			if (debug)	std::cout << (ii + 1) * timeStep << "\t" << (ii + 1) * FPGAns::convertUsTotick(timeStep) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
			getchar();
	}

	//Preset the output of the FPGA
	void RTsequence::presetFPGAoutput() const
	{
		//Disable pushing data to FIFOOUTfpga on the FPGA
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTfpgaEnable, 0));

		QU32 allQueues;		//Create a single long queue

		//Pixelclock. Push the entire pixelclock to avoid any conflict when triggering the digital and analog channels
		allQueues.push_back(mVectorOfQueues.at(PIXELCLOCK).size());
		for (int iter = 0; iter < static_cast<int>(mVectorOfQueues.at(PIXELCLOCK).size()); iter++)
			allQueues.push_back(mVectorOfQueues.at(PIXELCLOCK).at(iter));
			
		//allQueues.push_back(0);

		//Do the rest of the channels
		for (int chan = 1; chan < nChan; chan++)
		{
			const int sizePerChannel = static_cast<int>(mVectorOfQueues.at(chan).size());
			if (sizePerChannel == 0)
				allQueues.push_back(0);									//If the individual queue is empty
			else
			{
				allQueues.push_back(1);									//Push the number of elements in each individual queue, i.e, only 1 element
				allQueues.push_back(mVectorOfQueues.at(chan).at(0));	//Push the first element in VectorOfQueues[i]
			}

		}

		const int sizeFIFOINqueue = allQueues.size();		//Total number of elements in all the queues 

		if (sizeFIFOINqueue > FIFOINmax)
			throw std::overflow_error((std::string)__FUNCTION__ + ": FIFOIN overflow");

		std::vector<U32> FIFOIN(sizeFIFOINqueue);				//Create a 1D array with the channels concatenated
		for (int ii = 0; ii < sizeFIFOINqueue; ii++)
		{
			FIFOIN[ii] = allQueues.front();						//Transfer the queue elements to the array
			allQueues.pop_front();
		}
		allQueues = {};					//Cleanup the queue C++11 style

		const U32 timeout_ms = -1;		//in ms. A value -1 prevents FIFOIN from timing out
		U32 r;							//Elements remaining

		//Send the data to the FPGA through FIFOIN. I measured a minimum time of 10 ms to execute
		checkStatus(__FUNCTION__, NiFpga_WriteFifoU32(mFpga.getFpgaHandle(), NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, &FIFOIN[0], sizeFIFOINqueue, timeout_ms, &r));

		//On the FPGA, transfer the commands from FIFOIN to the sub-channel buffers
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));

		triggerRT();

		Sleep(100); //Wait long enough so that the sequence above does not wash out the following sequence

		//Set back 'NiFpga_FPGAvi_ControlBool_FIFOOUTfpgaEnable' to the original state on the FPGA
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOOUTfpgaEnable, FIFOOUTfpgaEnable));

	}

	//Send every single queue in 'mVectorOfQueue' to the FPGA buffer
	//For this, concatenate all the single queues 'VectorOfQueues.at(ii)' in the single queue 'allQueues'.
	//The data structure is allQueues = [# elements ch1| elements ch1 | # elements ch 2 | elements ch 2 | etc]. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL
	//Then, transfer all the elements in 'allQueues' to the vector FIFOIN to interface the FPGA
	void RTsequence::uploadRT() const
	{
		QU32 allQueues;		//Create a single long queue
		for (int chan = 0; chan < nChan; chan++)
		{
			allQueues.push_back(mVectorOfQueues.at(chan).size());	//Push the number of elements in each individual queue ii, 'VectorOfQueues.at(ii)'			
			for (int iter = 0; iter < static_cast<int>(mVectorOfQueues.at(chan).size()); iter++)
				allQueues.push_back(mVectorOfQueues.at(chan).at(iter));	//Push VectorOfQueues[i]
		}

		const int sizeFIFOINqueue = allQueues.size();		//Total number of elements in all the queues 

		if (sizeFIFOINqueue > FIFOINmax)
			throw std::overflow_error((std::string)__FUNCTION__ + ": FIFOIN overflow");

		std::vector<U32> FIFOIN(sizeFIFOINqueue);				//Create a 1D array with the channels concatenated
		for (int ii = 0; ii < sizeFIFOINqueue; ii++)
		{
			FIFOIN[ii] = allQueues.front();						//Transfer the queue elements to the array
			allQueues.pop_front();
		}
		allQueues = {};					//Cleanup the queue C++11 style

		const U32 timeout_ms = -1;		//in ms. A value -1 prevents FIFOIN from timing out
		U32 r;							//Elements remaining

		//Send the data to the FPGA through FIFOIN. I measured a minimum time of 10 ms to execute
		checkStatus(__FUNCTION__, NiFpga_WriteFifoU32(mFpga.getFpgaHandle(), NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, &FIFOIN[0], sizeFIFOINqueue, timeout_ms, &r));

		//On the FPGA, transfer the commands from FIFOIN to the sub-channel buffers
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	}

	//Execute the commands
	void RTsequence::triggerRT() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_LinegateTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));
	}

#pragma endregion "RTsequence"

}//namespace


/* Functions for generating a non-uniform pixel clock

extern const double RSpkpk_um = 250 * um;					//Peak-to-peak amplitude of the resonant scanner. Needed for generating a non-uniform pixelclock

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
mPixelclockQ.push_back(FPGAns::packU32(InitialWaitingTime_tick - mLatency_tick, 0));

//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)
mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(calculatePracticalDwellTime_us(pix), 1));

//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(tMIN_us, 1));
}

*/