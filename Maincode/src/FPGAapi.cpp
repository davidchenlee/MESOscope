#include "FPGAapi.h"

namespace FPGAapi
{
	//Convert microseconds to ticks
	U16 convertUs2tick(const double t_us)
	{
		const double t_tick = t_us * tickPerUs;

		if ((U32)t_tick > 0x0000FFFF)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step overflow. Time step cast to the max: " << std::fixed << _UI16_MAX * usPerTick << " us" << std::endl;
			return _UI16_MAX;
		}
		else if ((U32)t_tick < t_tick_MIN)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step underflow. Time step cast to the min: " << std::fixed << t_tick_MIN * usPerTick << " us" << std::endl;;
			return t_tick_MIN;
		}
		else
			return (U16)t_tick;
	}

	//Convert voltage to I16 [1]
	I16 convertVolt2I16(const double voltage_V)
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

	//Pack t in MSB and x in LSB. Time t and analog output x are encoded in 16 bits each.
	U32 packU32(const U16 t_tick, const U16 val)
	{
		return (t_tick << 16) | (0x0000FFFF & val);
	}

	//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't_us'
	U32 packAnalogSinglet(const double t_us, const double val)
	{
		const U16 AOlatency_tick = 2;	//To calibrate, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(convertUs2tick(t_us) - AOlatency_tick, convertVolt2I16(val));
	}


	//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't_us'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
	U32 packDigitalSinglet(const double t_us, const bool DO)
	{
		const U16 DOlatency_tick = 2;	//To calibrate, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(convertUs2tick(t_us) - DOlatency_tick, (U16)DO);
	}

	//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't_us'
	U32 packPixelclockSinglet(const double t_us, const bool DO)
	{
		const U16 PixelclockLatency_tick = 1;//The pixel-clock is implemented using a SCTL. I think the latency comes from reading the LUT buffer
		return packU32(convertUs2tick(t_us) - PixelclockLatency_tick, (U16)DO);
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
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));											//control-sequence trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));										//data-acquisition trigger
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)nChan));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16)syncDOtoAO_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate, (U16)syncAODOtoLinegate_tick));
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(nLinesAllFrames + nFrames * nLinesSkip)));		//Total number of lines in all the frames, including the skipped lines
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)heightPerFrame_pix));							//Number of lines in a frame, without including the skipped lines
		checkStatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(heightPerFrame_pix + nLinesSkip)));	//Number of lines in a frame including the skipped lines

		//Shutters
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

		//Vibratome control
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_start, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_back, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

		//Resonant scanner
		//checkStatus(__FUNCTION__,  NiFpga_WriteI16(mSession, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));	//Output voltage
		//checkStatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));	//Turn on/off

		//Debugger
		checkStatus(__FUNCTION__, NiFpga_WriteArrayBool(mSession, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, nPulses));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));	//FIFO OUT
	}

	//Send every single queue in VectorOfQueue to the FPGA buffer
	//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
	//Then transfer the elements in the long queue to an array to interface the FPGA
	//Improvement: the single queues VectorOfQueues[i] could be transferred directly to the FIFO array
	void Session::writeFIFO(VQU32 &vectorQueues) const
	{
		QU32 allQueues;											//Create a single long queue
		for (int i = 0; i < nChan; i++)
		{
			allQueues.push_back(vectorQueues[i].size());		//Push the number of elements in VectorOfQueues[i] (individual queue)

																//New version: Non-destructive. Randomly access the elements in VectorOfQueues[i] and push them to allQueues
			for (size_t iter = 0; iter < vectorQueues[i].size(); iter++)
				allQueues.push_back(vectorQueues[i].at(iter));

			/*Old version. Destructive
			while (!vectorQueues[i].empty())
			{
			allQueues.push_back(vectorQueues[i].front());	//Push all the elements in VectorOfQueues[i] to allQueues
			vectorQueues[i].pop_front();
			}
			*/
		}

		const int sizeFIFOqueue = allQueues.size();		//Total number of elements in all the queues 

		if (sizeFIFOqueue > FIFOINmax)
			throw std::overflow_error((std::string)__FUNCTION__ + ": FIFO IN overflow");

		U32* FIFO = new U32[sizeFIFOqueue];				//Create an array for interfacing the FPGA	
		for (int i = 0; i < sizeFIFOqueue; i++)
		{
			FIFO[i] = allQueues.front();				//Transfer the queue elements to the array
			allQueues.pop_front();
		}
		allQueues = {};									//Cleanup the queue (C++11 style)

		const U32 timeout_ms = -1;		// in ms. A value -1 prevents the FIFO from timing out
		U32 r;						//empty elements remaining

		checkStatus(__FUNCTION__, NiFpga_WriteFifoU32(mSession, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout_ms, &r)); //Send the data to the FPGA through the FIFO

		delete[] FIFO;		//cleanup the array
	}

	//Execute the commands
	void Session::triggerRT() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));
	}

	//Flush the block RAMs used for buffering the pixelclock, AO, and DO 
	void Session::flushBRAMs() const
	{
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
		//std::cout << "flushBRAMs called\n";
	}

	//The Session object has to be closed explicitly (in opposition to using the destructor) because it lives in main()
	void Session::close(const bool reset) const
	{
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
		case uniform: uniformDwellTimes();
			break;
		case corrected: correctedDwellTimes();
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
			return halfPeriodLineclock_us * asin(arg) / Constants::PI; //The returned value in in the range [-halfPeriodLineclock_us/PI, halfPeriodLineclock_us/PI]
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
	void RTsequence::Pixelclock::uniformDwellTimes()
	{
		//Relative delay of the pixel clock wrt the line clock (assuming perfect laser alignment, which is generally not true)
		//DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO
		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
		const double initialWaitingTime_us = 6.25*us;
		pixelclockQ.push_back(FPGAapi::packU32(FPGAapi::convertUs2tick(initialWaitingTime_us) - mLatency_tick, 0));

		//Generate the pixel clock. When HIGH is pushed, the pixel clock switches its state to represent a pixel delimiter
		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		const double dwellTime_us = 0.125 * us;
		for (int pix = 0; pix < widthPerFrame_pix + 1; pix++)
			pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(dwellTime_us, 1));
	}

	//Pixel clock sequence. Every pixel is equally spaced.
	//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
	void RTsequence::Pixelclock::correctedDwellTimes()
	{
		if (widthPerFrame_pix % 2 != 0)	//Throw exception if odd. Odd number of pixels not supported yet
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Odd number of pixels in the image width currently not supported");

		//Relative delay of the pixel clock with respect to the line clock. DO NOT use packDigitalSinglet because the pixelclock has a different latency from DO
		const U16 InitialWaitingTime_tick = (U16)(calibCoarse_tick + calibFine_tick);
		pixelclockQ.push_back(FPGAapi::packU32(InitialWaitingTime_tick - mLatency_tick, 0));

		//Generate the pixel clock. When a HIGH is pushed, the pixel clock switches its state to represent a pixel delimiter (the switching is implemented on the FPGA)
		for (int pix = -widthPerFrame_pix / 2; pix < widthPerFrame_pix / 2; pix++)
			pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(calculatePracticalDwellTime_us(pix), 1));

		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
		pixelclockQ.push_back(FPGAapi::packPixelclockSinglet(t_us_MIN, 1));
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


	void RTsequence::pushDigitalSinglet(const RTchannel chan, double t_us, const bool DO)
	{
		mVectorOfQueues.at(chan).push_back(FPGAapi::packDigitalSinglet(t_us, DO));
	}

	void RTsequence::pushAnalogSinglet(const RTchannel chan, const double t_us, const double AO)
	{
		if (t_us < AO_t_us_MIN)
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_t_us_MIN << " us" << std::endl;

		mVectorOfQueues.at(chan).push_back(FPGAapi::packAnalogSinglet(AO_t_us_MIN, AO));
	}

	void RTsequence::pushLinearRamp(const RTchannel chan, double timeStep_us, const double rampDuration, const double Vinitial, const double Vfinal)
	{
		const bool debug = 0;

		if (timeStep_us < AO_t_us_MIN)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AO_t_us_MIN << " us" << std::endl;
			timeStep_us = AO_t_us_MIN;		//Analog output time increment (in us)
		}

		const int nPoints = (int)(rampDuration / timeStep_us);		//Number of points

		if (nPoints <= 1)	throw std::invalid_argument((std::string)__FUNCTION__ + ": Not enought points to generate a linear ramp");

		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}

		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V = Vinitial + (Vfinal - Vinitial)*ii / (nPoints - 1);
			mVectorOfQueues.at(chan).push_back(FPGAapi::packAnalogSinglet(timeStep_us, V));

			if (debug)	std::cout << (ii + 1) * timeStep_us << "\t" << (ii + 1) * FPGAapi::convertUs2tick(timeStep_us) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
			getchar();

	}

	//Upload the commands to the FPGA (see the implementation of the LV code), but do not execute yet
	void  RTsequence::uploadRT()
	{
		mFpga.writeFIFO(mVectorOfQueues);

		//On the FPGA, transfer the commands from FIFO IN to the sub-channel buffers
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
		checkStatus(__FUNCTION__, NiFpga_WriteBool(mFpga.getSession(), NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
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

