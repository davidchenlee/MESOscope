#include "FPGAapi.h"

namespace GenericFPGAfunctions {

	void printHex(int input)
	{
		std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
	}
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
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step overflow. Time step cast to the max: " << std::fixed << _UI16_MAX * dt_us << " us" << std::endl;
			return _UI16_MAX;
		}
		else if ((U32)t_tick < dtMIN_tick)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step underflow. Time step cast to the min: " << std::fixed << dtMIN_tick * dt_us << " us" << std::endl;;
			return dtMIN_tick;
		}
		else
			return (U16)t_tick;
	}

	//Convert voltage to I16 [1]
	I16 convertVolt2I16(double x)
	{
		const int VMAX = 10 * V;
		const int VMIN = -10 * V;

		if (x > 10)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage overflow. Voltage cast to the max: " + std::to_string(VMAX) + " V" << std::endl;
			return (I16)_I16_MAX;
		}
		else if (x < -10)
		{
			std::cerr << "WARNING in " << __FUNCTION__ << ": Voltage underflow. Voltage cast to the min: " + std::to_string(VMIN) + " V" << std::endl;
			return (I16)_I16_MIN;
		}
		else
			return (I16)(x / 10 * _I16_MAX);
	}


	//Send out an analog instruction, where the analog level 'val' is held for the amount of time 't'
	U32 packSingleAnalog(double t, double val)
	{
		const U16 AOlatency_tick = 2;	//To calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		return packU32(convertUs2tick(t) - AOlatency_tick, convertVolt2I16(val));
	}


	//Send out a single digital instruction, where 'DO' is held LOW or HIGH for the amount of time 't'. The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
	U32 packSingleDigital(double t, bool DO)
	{
		const U16 DOlatency_tick = 2;	//To calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles to read
		if (DO)
			return packU32(convertUs2tick(t) - DOlatency_tick, 0x0001);
		else
			return packU32(convertUs2tick(t) - DOlatency_tick, 0x0000);
	}


	//Generate a single pixel-clock instruction, where 'DO' is held LOW or HIGH for the amount of time 't'
	U32 packSinglePixelclock(double t, bool DO)
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
			std::cerr << "WARNING in " << __FUNCTION__ << ": Time step too small. Time step cast to " << AOdt_us << " us" << std::endl;
			TimeStep = AOdt_us;						//Analog output time increment (in us)
		}

		const int nPoints = (int)(RampLength / TimeStep);		//Number of points

		if (nPoints <= 1)	throw std::invalid_argument((std::string)__FUNCTION__ + ": Not enought points to generate a linear ramp");

		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}

		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V = Vinitial + (Vfinal - Vinitial)*ii / (nPoints - 1);
			queue.push_back(packSingleAnalog(TimeStep, V));

			if (debug)	std::cout << (ii + 1) * TimeStep << "\t" << (ii + 1) * convertUs2tick(TimeStep) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
		{
			getchar();
			return {};
		}

		return queue;
	}

}//namespace


FPGAapi::FPGAapi()
{
	//Must be called before any other FPGA calls
	checkFPGAstatus(__FUNCTION__, NiFpga_Initialize());

	//Opens a session, downloads the bitstream. 1=no run, 0=run
	checkFPGAstatus(__FUNCTION__, NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &mSession));
}

FPGAapi::~FPGAapi()
{
	//std::cout << "FPGAapi destructor was called" << std::endl;
};

void FPGAapi::initialize() const
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_PhotoncounterInputSelector, photoncounterInput));				//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_LineclockInputSelector, lineclockInput));					//Select the Line clock: resonant scanner or function generator
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));									//control-sequence trigger
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));									//data-acquisition trigger
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout_tick));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)Nchan));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16)syncDOtoAO_tick));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate, (U16)syncAODOtoLinegate_tick));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(nLinesAllFrames + nFrames * nLinesSkip)));			//Total number of lines in all the frames, including the skipped lines
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)heightPerFrame_pix));							//Number of lines in a frame, without including the skipped lines
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(heightPerFrame_pix + nLinesSkip)));	//Number of lines in a frame including the skipped lines

	//Shutters
	//checkFPGAstatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
	//checkFPGAstatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

	//Vibratome control
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Resonant scanner
	//checkFPGAstatus(__FUNCTION__,  NiFpga_WriteI16(mSession, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));	//Output voltage
	//checkFPGAstatus(__FUNCTION__,  NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));	//Turn on/off

	//PockelsID cells
	checkFPGAstatus(__FUNCTION__,  NiFpga_WriteI16(mSession, NiFpga_FPGAvi_ControlI16_PC1_voltage, 0));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_PC1_selectTrigger, 0));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_PC1_manualOn, 0));

	//Debugger
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteArrayBool(mSession, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, nPulses));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));	//FIFO OUT
}

//Send every single queue in VectorOfQueue to the FPGA buffer
//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
//Then transfer the elements in the long queue to an array to interface the FPGA
//Improvement: the single queues VectorOfQueues[i] could be transferred directly to the FIFO array
void FPGAapi::writeFIFO(VQU32 &vectorQueues) const
{
	QU32 allQueues;											//Create a single long queue
	for (int i = 0; i < Nchan; i++)
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

	checkFPGAstatus(__FUNCTION__, NiFpga_WriteFifoU32(mSession, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout_ms, &r)); //Send the data to the FPGA through the FIFO

	delete[] FIFO;		//cleanup the array
}

//Execute the commands
void FPGAapi::triggerRT() const
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 1));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));
}

//Trigger the FIFO flushing
void FPGAapi::flushFIFO() const
{
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1));
	checkFPGAstatus(__FUNCTION__, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
}

//The FPGAapi object has to be closed explicitly (in opposition to using the destructor) because it lives in main()
void FPGAapi::close(const bool reset) const
{
	//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
	//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
	//0 resets, 1 does not reset
	checkFPGAstatus(__FUNCTION__, NiFpga_Close(mSession, (U32)!reset));

	//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
	checkFPGAstatus(__FUNCTION__, NiFpga_Finalize());
}

NiFpga_Session FPGAapi::getSession() const
{
	return mSession;
}

void checkFPGAstatus(char functionName[], NiFpga_Status status)
{
	if (status < 0)
		throw FPGAexception((std::string)functionName + " with FPGA code " + std::to_string(status));
	if (status > 0)
		std::cerr << "A warning has ocurred in " << functionName << " with FPGA code " << status << std::endl;
}





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

