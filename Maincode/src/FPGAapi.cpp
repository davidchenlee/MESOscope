#include "FPGAapi.h"

namespace FPGAns
{
	//Convert time to ticks
	U16 timeToTick(const double t)
	{
		const double t_tick{ t / us * tickPerUs };

		if (static_cast<U32>(t_tick) > 0x0000FFFF)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step overflow. Time step cast to the max: " << std::fixed << _UI16_MAX * usPerTick << " us\n";
			return _UI16_MAX;
		}
		else if (static_cast<U32>(t_tick) < tMIN_tick)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step underflow. Time step cast to the min: " << std::fixed << tMIN_tick * usPerTick << " us\n";
			return tMIN_tick;
		}
		else
			return static_cast<U16>(t_tick);
	}

	//Convert voltage (-10V to 10V) to I16 (-32768 to 32767)
	//Examples of I16 numbers:
	//0x7FFF = 32767
	//0xFFFF = -1
	//0x8000 = -32768
	I16 voltageToI16(const double voltage)
	{
		if (voltage > AOmax)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage overflow. Voltage cast to the max: " + std::to_string(AOmax/V) + " V\n";
			return (I16)_I16_MAX;
		}
		else if (voltage < -AOmax)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage underflow. Voltage cast to the min: " + std::to_string(-AOmax/V) + " V\n";
			return (I16)_I16_MIN;
		}
		else
			return (I16)(voltage / AOmax * _I16_MAX);
	}

	//Convert I16 (-32768 to 32767) to voltage (-10V to 10V)
	double I16toVoltage(const int input)
	{
		//Positive case
		if (input >= 0)
		{
			//Check for overflow
			if (input > _I16_MAX)
			{
				std::cerr << "WARNING in " << __FUNCTION__ << ": Input overflow, _I16_MAX used instead\n";
				return AOmax/V;
			}
			else
				return 1. * input / _I16_MAX * AOmax;
		}
		else //Negative case
		{
			//Check for underoverflow
			if (input < _I16_MIN)
			{
				std::cerr << "WARNING in " << __FUNCTION__ << ": Input underflow, _I16_MIN used instead\n";
				return -AOmax/V;
			}
			else
				return -1. * input / _I16_MIN * AOmax;
		}		
	}


	//Pack t in MSB and x in LSB. Time t and analog output AO_U16 are encoded in 16 bits each.
	U32 packU32(const U16 t_tick, const U16 AO_U16)
	{
		return (t_tick << 16) | (0x0000FFFF & AO_U16);
	}

	//Send out an analog instruction, where the analog output 'AO' is held for 'timeStep'
	U32 packAnalogSinglet(const double timeStep, const double AO)
	{
		const U16 AOlatency_tick{ 2 };	//To calibrate, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(timeToTick(timeStep) - AOlatency_tick, voltageToI16(AO/V));
	}

	//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 'timeStep'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
	U32 packDigitalSinglet(const double timeStep, const bool DO)
	{
		const U16 DOlatency_tick{ 2 };	//To calibrate, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(timeToTick(timeStep) - DOlatency_tick, static_cast<U16>(DO));
	}

	//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 'timeStep'
	U32 packPixelclockSinglet(const double timeStep, const bool DO)
	{
		const U16 PixelclockLatency_tick{ 1 };//The pixel-clock is implemented using a SCTL. I think the latency comes from reading the LUT buffer
		return packU32(timeToTick(timeStep) - PixelclockLatency_tick, static_cast<U16>(DO));
	}

	void checkStatus(char functionName[], NiFpga_Status status)
	{
		if (status < 0)
			throw  FPGAns::FPGAexception((std::string)functionName + " with FPGA code " + std::to_string(status));
		if (status > 0)
			std::cerr << "A warning has ocurred in " << functionName << " with FPGA code " << status << "\n";
	}

	void linearRamp(QU32 &queue, double timeStep, const double rampLength, const double Vi, const double Vf)
	{
		if (timeStep < AO_tMIN)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_tMIN / us << " us\n";
			timeStep = AO_tMIN;		//Analog Out time increment in us
		}

		const int nPoints{ static_cast<int>(rampLength / timeStep) };		//Number of points

		if (nPoints <= 1)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Not enought points to generate a linear ramp");

		//For debugging
		//std::cout << "nPoints: " << nPoints << "\n";
		//std::cout << "time \tticks \tv\n";

		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V{ Vi + (Vf - Vi)*ii / (nPoints - 1) };
			queue.push_back(FPGAns::packAnalogSinglet(timeStep, V));

			//std::cout << (ii + 1) * timeStep << "\t" << (ii + 1) * timeToTick(timeStep) << "\t" << V << "\t\n";	//For debugging
		}
		//getchar();	//For debugging
	}

#pragma region "FPGA"
	FPGA::FPGA()
	{
		//Must be called before any other FPGA calls
		checkStatus(__FUNCTION__, NiFpga_Initialize());

		//Opens a session, uploads the bitfile to the FPGA. 1=no run, 0=run
		checkStatus(__FUNCTION__, NiFpga_Open(mBitfile.c_str(), NiFpga_FPGAvi_Signature, "RIO0", 0, &mHandle));

		//Set up the FPGA parameters
		initializeFpga_();
	}

	FPGA::~FPGA()
	{
		//std::cout << "FPGA destructor was called\n";
	};

	//The object has to be closed explicitly because of the exception catching
	void FPGA::close(const FPGARESET reset) const
	{
		//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
		//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
		//0 resets, 1 does not reset
		checkStatus(__FUNCTION__, NiFpga_Close(mHandle, !static_cast<bool>(reset)));

		if (static_cast<bool>(reset))
			std::cout << "The FPGA has been successfully reset\n";

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		checkStatus(__FUNCTION__, NiFpga_Finalize());
	}

	NiFpga_Session FPGA::getHandle() const
	{
		return mHandle;
	}

	//Load the imaging parameters onto the FPGA. See 'Const.cpp' for the definition of each variable
	void FPGA::initializeFpga_() const
	{
		if (FIFOtimeout_tick < 0 || syncDOtoAO_tick < 0 || pockelsFirstFrameDelay < 0 || pockelsSecondaryDelay < 0  || galvosCommonDelay < 0 || rescanGalvoDelay < 0 || linegateTimeout < 0 || stagePulseStretcher < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": One or more imaging parameters take negative values");

		//INPUT SELECTORS
		checkStatus(__FUNCTION__, NiFpga_WriteU8(getHandle(), NiFpga_FPGAvi_ControlU8_PhotocounterInputSelector, static_cast<U8>(photocounterInput)));						//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
		checkStatus(__FUNCTION__, NiFpga_WriteArrayBool(getHandle(), NiFpga_FPGAvi_ControlArrayBool_PulseSequence, pulseArray, nPulses));									//For debugging the photocounters

		//FIFOIN
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getHandle(), NiFpga_FPGAvi_ControlU16_Nchannels, static_cast<U16>(RTCHAN::NCHAN)));												//Number of input channels
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, false));															//Trigger of the control sequence
		checkStatus(__FUNCTION__, NiFpga_WriteI32(getHandle(), NiFpga_FPGAvi_ControlI32_FIFOtimeout_tick, static_cast<I32>(FIFOtimeout_tick)));								//FIFOIN timeout

		//TRIGGERS AND DELAYS
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_PcTrigger, false));																				//Pc trigger signal
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable, false));																	//Z-stage as tigger
		checkStatus(__FUNCTION__, NiFpga_WriteU16(getHandle(), NiFpga_FPGAvi_ControlU16_SyncDOtoAO_tick, static_cast<U16>(syncDOtoAO_tick)));												//DO and AO relative sync
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_PockelsFirstFrameDelay_tick, static_cast<U32>(pockelsFirstFrameDelay / us * tickPerUs)));			//Pockels delay wrt the preframeclock
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_PockelsFrameDelay_tick, static_cast<U32>(pockelsSecondaryDelay / us * tickPerUs)));					//Pockels delay wrt the preframeclock
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_ScanGalvoDelay_tick, static_cast<U32>(galvosCommonDelay / us * tickPerUs)));						//Scan galvo delay wrt the preframeclock
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_RescanGalvoDelay_tick, static_cast<U32>((galvosCommonDelay + rescanGalvoDelay) / us * tickPerUs)));	//Rescan galvo delay wrt the preframeclock
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_TriggerAODOexternal, false));																		//Trigger the FPGA outputs (non-RT trigger)
		checkStatus(__FUNCTION__, NiFpga_WriteI16(getHandle(), NiFpga_FPGAvi_ControlI16_Npreframes, static_cast<I16>(nPreframes)));															//Number of lineclocks separating the preframeclock(preframegate) and the frameclock (framegate)

		if (linegateTimeout <= 2 * halfPeriodLineclock)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The linegate timeout must be greater than the lineclock period");
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_LinegateTimeout_tick, static_cast<U32>(linegateTimeout / us * tickPerUs)));			//Timeout the trigger of the control sequence

		//POCKELS CELLS
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_PockelsAutoOffEnable, pockelsAutoOff));											//Enable gating the pockels by framegate. For debugging purposes

		//VIBRATOME
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_VTstart, false));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_VTback, false));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_VTforward, false));

		//STAGES
		checkStatus(__FUNCTION__, NiFpga_WriteU32(getHandle(), NiFpga_FPGAvi_ControlU32_StagePulseStretcher_tick, static_cast<U32>(stagePulseStretcher / us * tickPerUs)));	//Stretch the pulsewidth from the stages. Currently not in use
		
		//Flush the RAM buffers on the FPGA as precaution. 
		checkStatus(__FUNCTION__, NiFpga_WriteBool(getHandle(), NiFpga_FPGAvi_ControlBool_FlushTrigger, false));															//Memory-flush trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mHandle, NiFpga_FPGAvi_ControlBool_FlushTrigger, true));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mHandle, NiFpga_FPGAvi_ControlBool_FlushTrigger, false));
		//std::cout << "flushBRAMs called\n";	//For debugging

		/*
		//SHUTTERS. Commented out to allow keeping the shutter on
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_Shutter1, false));
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_Shutter2, false));

		//RESONANT SCANNER. Commented out to allow keeping the RS on
		checkStatus(__FUNCTION__,  NiFpga_WriteI16(mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_RScontrol_I16, false));	//Output voltage
		checkStatus(__FUNCTION__,  NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_RSenable, false));	//Turn on/off
		*/
	}
#pragma endregion "FPGA"

#pragma region "RTcontrol"
	RTcontrol::Pixelclock::Pixelclock(const int widthPerFrame_pix, const double dwell) : mWidthPerFrame_pix(widthPerFrame_pix), mDwell(dwell)
	{
		const int calibFine_tick{ -40 };
		switch (pixelclockType)
		{
		case PIXELCLOCK::UNIFORM:
			pushUniformDwellTimes(calibFine_tick);
			break;
		//case nonuniform: pushCorrectedDwellTimes();
			//break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected pixelclock type unavailable");
		}
	}

	//Pixelclock with equal dwell times
	//calibFine_tick: fine tune the pixelclock timing
	void RTcontrol::Pixelclock::pushUniformDwellTimes(const int calibFine_tick)
	{
		//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime'. At 160MHz, the clock increment is 6.25ns = 0.00625us
		//For example, for a dwell time = 125ns and 400 pixels, the initial waiting time is (halfPeriodLineclock-400*125ns)/2

		const double initialWaitingTime{ (halfPeriodLineclock - mWidthPerFrame_pix * mDwell) / 2 }; //Relative delay of the pixel clock wrt the line clock

		//Check if the pixelclock overflows each Lineclock
		if (initialWaitingTime <= 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Pixelclock overflow");

		mPixelclockQ.push_back(FPGAns::packU32(FPGAns::timeToTick(initialWaitingTime) + calibFine_tick - mLatency_tick, 0));	 //DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO

		//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		for (int pix = 0; pix < mWidthPerFrame_pix + 1; pix++)
			mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(mDwell, 1));
	}

	QU32 RTcontrol::Pixelclock::readPixelclock() const
	{
		return mPixelclockQ;
	}

	RTcontrol::RTcontrol(const FPGAns::FPGA &fpga, const LINECLOCK lineclockInput, const MAINTRIG mainTrigger, const int nFrames, const int widthPerFrame_pix, const int heightPerFrame_pix, FIFOOUT FIFOOUTstate) :
		mVectorOfQueues(static_cast<U8>(RTCHAN::NCHAN)), mFpga(fpga), mLineclockInput(lineclockInput), mMainTrigger(mainTrigger), mNframes(nFrames), mWidthPerFrame_pix(widthPerFrame_pix), mHeightPerFrame_pix(heightPerFrame_pix), mFIFOOUTstate(FIFOOUTstate)
	{
		//Set the imaging parameters
		mHeightAllFrames_pix = mHeightPerFrame_pix * mNframes;
		mNpixAllFrames = mWidthPerFrame_pix * mHeightAllFrames_pix;
		uploadImagingParameters_();

		//Generate a pixelclock
		const Pixelclock pixelclock(mWidthPerFrame_pix, mDwell);
		mVectorOfQueues.at(static_cast<U8>(RTCHAN::PIXELCLOCK)) = pixelclock.readPixelclock();

		//If the z stage acts as the main trigger (for cont z scanning), add a timer after the sequence ends because the motion monitor of the z stage bounces and false-triggers the acq sequence
		if (mMainTrigger == MAINTRIG::ZSTAGE)
		{
			checkStatus(__FUNCTION__, NiFpga_WriteU32(mFpga.getHandle(), NiFpga_FPGAvi_ControlU32_PostsequenceTimer_tick, static_cast<U32>(postsequenceTimer / us * tickPerUs)));
			//std::cout << "Z stage as the main trigger\n";
		}
		else
		{
			checkStatus(__FUNCTION__, NiFpga_WriteU32(mFpga.getHandle(), NiFpga_FPGAvi_ControlU32_PostsequenceTimer_tick, 0));
			//std::cout << "PC as the main trigger\n";
		}
	}

	RTcontrol::~RTcontrol(){}

	//Load the imaging parameters onto the FPGA
	void RTcontrol::uploadImagingParameters_() const
	{
		if (mNframes < 0 || mHeightAllFrames_pix < 0 || mHeightPerFrame_pix < 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": One or more imaging parameters take negative values");

		checkStatus(__FUNCTION__, NiFpga_WriteI16(mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_Nframes, static_cast<I16>(mNframes)));						//Number of frames to acquire
		checkStatus(__FUNCTION__, NiFpga_WriteI32(mFpga.getHandle(), NiFpga_FPGAvi_ControlI32_NlinesAll, static_cast<I32>(mHeightAllFrames_pix)));			//Total number of lines in all the frames
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mFpga.getHandle(), NiFpga_FPGAvi_ControlI16_NlinesPerFrame, static_cast<I16>(mHeightPerFrame_pix)));		//Number of lines in a frame
	
		//SELECTORS
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_LineclockInputSelector, static_cast<bool>(mLineclockInput)));					//Lineclock: resonant scanner (RS) or function generator (FG)
	}


	//Send every single queue in 'vectorOfQueue' to the FPGA buffer
	//For this, concatenate all the individual queues 'vectorOfQueuesForRamp.at(ii)' in the queue 'allQueues'.
	//The data structure is allQueues = [# elements ch1| elements ch1 | # elements ch 2 | elements ch 2 | etc]. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL
	//Then transfer all the elements in 'allQueues' to the vector FIFOIN to interface the FPGA
	void RTcontrol::uploadFIFOIN_(const VQU32 &vectorOfQueues) const
	{
		{
			QU32 allQueues;		//Create a single long queue
			for (int chan = 0; chan < static_cast<U8>(RTCHAN::NCHAN); chan++)
			{
				allQueues.push_back(vectorOfQueues.at(chan).size());	//Push the number of elements in each individual queue ii, 'VectorOfQueues.at(ii)'	
				for (std::vector<int>::size_type iter = 0; iter != vectorOfQueues.at(chan).size(); iter++)
					allQueues.push_back(vectorOfQueues.at(chan).at(iter));	//Push VectorOfQueues[i]
			}

			const int sizeFIFOINqueue{ static_cast<int>(allQueues.size()) };	//Total number of elements in all the queues 

			if (sizeFIFOINqueue > FIFOINmax)
				throw std::overflow_error((std::string)__FUNCTION__ + ": FIFOIN overflow");

			std::vector<U32> FIFOIN(sizeFIFOINqueue);							//Create a 1D array with the channels concatenated
			for (int ii = 0; ii < sizeFIFOINqueue; ii++)
			{
				FIFOIN.at(ii) = allQueues.front();								//Transfer the queue elements to the array
				allQueues.pop_front();
			}
			allQueues = {};					//Cleanup the queue C++11 style

			U32 r;							//Elements remaining

			//Send the data to the FPGA through FIFOIN. I measured a minimum time of 10 ms to execute
			checkStatus(__FUNCTION__, NiFpga_WriteFifoU32(mFpga.getHandle(), NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, &FIFOIN[0], sizeFIFOINqueue, NiFpga_InfiniteTimeout, &r));

			//On the FPGA, transfer the commands from FIFOIN to the sub-channel buffers. 
			//This boolean serves as the master trigger for the entire control sequence
			checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, true));
			checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, false));
		}
	}
	
	//Trigger the FPGA outputs in a non-realtime way (see the LV implementation)
	void RTcontrol::triggerNRT_() const
	{	
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_TriggerAODOexternal, true));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_TriggerAODOexternal, false));
	}

	//Push all the elements in 'tailQ' into 'headQ'
	void RTcontrol::concatenateQueues_(QU32& receivingQueue, QU32& givingQueue) const
	{
		while (!givingQueue.empty())
		{
			receivingQueue.push_back(givingQueue.front());
			givingQueue.pop_front();
		}
	}

	void RTcontrol::pushQueue(const RTCHAN chan, QU32& queue)
	{
		concatenateQueues_(mVectorOfQueues.at(static_cast<U8>(chan)), queue);
	}

	void RTcontrol::clearQueue(const RTCHAN chan)
	{
		mVectorOfQueues.at(static_cast<U8>(chan)).clear();
	}

	void RTcontrol::pushDigitalSinglet(const RTCHAN chan, double timeStep, const bool DO)
	{
		mVectorOfQueues.at(static_cast<U8>(chan)).push_back(FPGAns::packDigitalSinglet(timeStep, DO));
	}

	void RTcontrol::pushAnalogSinglet(const RTCHAN chan, double timeStep, const double AO, const OVERRIDE override)
	{
		if (timeStep < AO_tMIN)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_tMIN / us << " us\n";
			timeStep = AO_tMIN;
		}

		//Clear the current content
		if (static_cast<bool>(override))
			mVectorOfQueues.at(static_cast<U8>(chan)).clear();

		mVectorOfQueues.at(static_cast<U8>(chan)).push_back(FPGAns::packAnalogSinglet(timeStep, AO));
	}

	//Push a fixed-point number. For scaling the pockels cell output
	void RTcontrol::pushAnalogSingletFx2p14(const RTCHAN chan, const double scalingFactor)
	{
		mVectorOfQueues.at(static_cast<U8>(chan)).push_back(static_cast<U32>(doubleToFx2p14(scalingFactor)));
	}

	void RTcontrol::pushLinearRamp(const RTCHAN chan, double timeStep, const double rampLength, const double Vi, const double Vf)
	{
		linearRamp(mVectorOfQueues.at(static_cast<U8>(chan)), timeStep, rampLength, Vi, Vf);
	}

	//Preset the FPGA output with the first value in the RT control sequence to avoid discontinuities at the start of the sequence
	void RTcontrol::presetFPGAoutput() const
	{
		//Read from the FPGA the last voltage in the galvo AO. See the LV implementation
		std::vector<I16> AOlastVoltage_I16(static_cast<U8>(RTCHAN::NCHAN), 0);
		checkStatus(__FUNCTION__, NiFpga_ReadI16(mFpga.getHandle(), NiFpga_FPGAvi_IndicatorU16_ScanGalvoMon, &AOlastVoltage_I16.at(static_cast<U8>(RTCHAN::SCANGALVO))));
		checkStatus(__FUNCTION__, NiFpga_ReadI16(mFpga.getHandle(), NiFpga_FPGAvi_IndicatorU16_RescanGalvoMon, &AOlastVoltage_I16.at(static_cast<U8>(RTCHAN::RESCANGALVO))));

		//Create a vector of queues
		VQU32 vectorOfQueuesForRamp{ static_cast<U8>(RTCHAN::NCHAN) };
		for (int chan = 1; chan < static_cast<U8>(RTCHAN::NCHAN); chan++) //chan > 0 means that the pixelclock is kept empty
		{
			if (mVectorOfQueues.at(chan).size() != 0)
			{
				//Linear ramp the output to smoothly transition from the end point of the previous run to the start point of the next run
				if ((chan == static_cast<U8>(RTCHAN::SCANGALVO) || chan == static_cast<U8>(RTCHAN::RESCANGALVO)) )	//Only do GALVO1 and GALVO2 for now
				{
					const double Vi = I16toVoltage(AOlastVoltage_I16.at(chan));				//Last element of the last RT control sequence
					const double Vf = I16toVoltage((I16)mVectorOfQueues.at(chan).front());	//First element of the new RT control sequence
		
					//For debugging
					//std::cout << Vi << "\n";
					//std::cout << Vf << "\n";

					linearRamp(vectorOfQueuesForRamp.at(chan), 10 * us, 5 * ms, Vi, Vf);
				}
			}
		}
		uploadFIFOIN_(vectorOfQueuesForRamp);		//Load the ramp on the FPGA
		triggerNRT_();								//Trigger the FPGA outputs (non-RT trigger)
	}

	void RTcontrol::uploadRT() const
	{
		uploadFIFOIN_(mVectorOfQueues);
	}

	//Trigger the RT control sequence on the FPGA
	void RTcontrol::triggerRT() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_PcTrigger, true));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getHandle(), NiFpga_FPGAvi_ControlBool_PcTrigger, false));
	}

#pragma endregion "RTcontrol"

}//namespace


/* Functions for generating a non-uniform pixel clock

extern const double RSpkpk_um = 250 * um;					//Peak-to-peak amplitude of the resonant scanner. Needed for generating a non-uniform pixelclock

//Convert the spatial coordinate of the resonant scanner to time. x in [-RSpkpk_um/2, RSpkpk_um/2]
double RTcontrol::Pixelclock::convertSpatialCoordToTime_us(const double x) const
{
double arg = 2 * x / RSpkpk_um;
if (arg > 1)
throw std::invalid_argument((std::string)__FUNCTION__ + ": Argument of asin greater than 1");
else
return halfPeriodLineclock / us * asin(arg) / Constants::PI; //The returned value is in the range [-halfPeriodLineclock / us/PI, halfPeriodLineclock / us/PI]
}

//Discretize the spatial coordinate, then convert it to time
double RTcontrol::Pixelclock::getDiscreteTime_us(const int pix) const
{
const double dx = 0.5 * um;
return convertSpatialCoordToTime_us(dx * pix);
}

//Calculate the dwell time for the pixel
double RTcontrol::Pixelclock::calculateDwellTime_us(const int pix) const
{
return getDiscreteTime_us(pix + 1) - getDiscreteTime_us(pix);
}

//Calculate the practical dwell time of each pixel, considering that the FPGA has discrete time steps
double RTcontrol::Pixelclock::calculatePracticalDwellTime_us(const int pix) const
{
return round(calculateDwellTime_us(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}


//Pixelclock with equal pixel size (spatial).
void RTcontrol::Pixelclock::pushCorrectedDwellTimes()
{
//The pixel clock is triggered by the line clock (see the LV implementation) followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
const int calibCoarse_tick = 2043;	//calibCoarse_tick: Look at the oscilloscope and adjust to center the pixel clock within a line scan
const int calibFine_tick = 10;

if (widthPerFrame_pix % 2 != 0)		//Throw exception if odd number of pixels (not supported yet)
throw std::invalid_argument((std::string)__FUNCTION__ + ": Odd number of pixels for the image width currently not supported");

//Relative delay of the pixel clock with respect to the line clock. DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO
const U16 InitialWaitingTime_tick = static_cast<U16>(calibCoarse_tick + calibFine_tick);
mPixelclockQ.push_back(FPGAns::packU32(InitialWaitingTime_tick - mLatency_tick, 0));

//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state, which corresponds to a pixel delimiter (boolean switching is implemented on the FPGA)
for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)
mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(calculatePracticalDwellTime_us(pix), 1));

//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
mPixelclockQ.push_back(FPGAns::packPixelclockSinglet(tMIN_us, 1));
}

*/