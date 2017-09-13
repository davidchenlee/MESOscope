#include "FPGA.h"


void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteI32(session, NiFpga_FPGA_ControlI32_FIFO_timeout, FIFOtimeout));
	NiFpga_MergeStatus(status, NiFpga_WriteI32(session, NiFpga_FPGA_ControlI32_Nchannels, Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteI32(session, NiFpga_FPGA_ControlI32_Ncounters, Ncounters));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_DO_to_AO, Sync_DO_to_AO));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_AODO_to_LineGate, Sync_AODO_to_LineGate));
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGA_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGA_ControlI16_Nmax_lines, Nmaxlines));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0)); //start acquiring data

	/*
	//Initialize all the channels with zero. Not needed if NiFpga_Finalize() is run at the end of the main code
	U32QV QV(Nchan);
	for (U8 ii = 0; ii < Nchan; ii++)
		QV[ii].push(0);
	SendOutQueue(status, session, QV);
	PulseTrigger(status, session);
	*/

	std::cout << "FPGA initialize-variables status: " << *status << "\n";
}


void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& QV)
{
	//take a vector of queues and return it as a single queue
	U32Q allQs;
	for (U32 i = 0; i < Nchan; i++)
	{
		allQs.push(QV[i].size()); //push the number of elements in the queue
		while (!QV[i].empty())
		{
			allQs.push(QV[i].front());
			QV[i].pop();
		}
	}
	//transfer the queue to an array to be sent to the FPGA. THE ORDER DETERMINES THE TARGETED CHANNEL
	U32 sizeFIFOqueue = allQs.size();
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

	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << "\n";
	delete[] FIFO;//cleanup
}

//Main trigger. Trigger FIFO-in, and therefore, AO and DO
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	std::cout << "Pulse trigger status: " << *status << "\n";
}

//Trigger the pixel clock, and therefore, counters, and FIFO-out
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0));
	std::cout << "Acquisition trigger status: " << *status << "\n";
}


//**************************************************************************************************************************************************************

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
		std::cout << "WARNING: time step overflow. Time step set to the max: " << std::fixed << _UI16_MAX * dt_us << " us\n";
		return _UI16_MAX;
	}
	else if ((U32)aux < dt_tick_MIN)
	{
		std::cout << "WARNING: time step underflow. Time step set to the min:" << std::fixed << dt_tick_MIN * dt_us << " us\n";;
		return dt_tick_MIN;
	}
	else
		return (U16)aux;
}


//converts voltage (range: -10V to 10V) to a signed int 16 (range: -32768 to 32767)
//0x7FFFF = 0d32767
//0xFFFF = -1
//0x8000 = -32768
I16 AOUT(double x)
{
	if (x > 10)
	{
		std::cout << "WARNING: voltage overflow. Voltage set to the max: 10 V\n";
		return (U16)_I16_MAX;
	}
	else if (x < -10)
	{
		std::cout << "WARNING: voltage underflow. Voltage set to the min: -10 V\n";
		return (U16)_I16_MIN;
	}
	else
		return (U16)(x / 10 * _I16_MAX);
}


U32 AnalogOut(double t, double val)
{
	U16 AOlatency_tick = 2;	//To  calibrate it, run AnalogLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	return u32pack(us2tick(t) - AOlatency_tick, AOUT(val));
}


//The DOs in Connector1 are rated at 10MHz, Connector0 at 80MHz.
U32 DigitalOut(double t, bool DO)
{
	U16 DOlatency_tick = 2;	//To  calibrate it, run DigitalLatencyCalib(). I think the latency comes from the memory block, which takes 2 cycles for reading
	if (DO)
		return u32pack(us2tick(t) - DOlatency_tick, 0x0001);
	else
		return u32pack(us2tick(t) - DOlatency_tick, 0x0000);
}


//Wait a certain amount of time once the pixel-clock is triggered by the resonant scanner. For this, send a package with a wait-time and zero bit
U32 PixelClockDelay(double t)
{
	U16 GateLatency_tick = 2;
	return u32pack(us2tick(t) - GateLatency_tick, 0x0000);
}

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
		std::cout << "WARNING: time step too small. Time step set to " << AOdt_us << " us\n";
		dt = AOdt_us; //Analog output time increment in us
		getchar();
	}

	U32 nPoints = (U32)(T / dt);		//number of points

	if (nPoints <= 1)
	{
		std::cout << "ERROR: not enought points for the linear ramp\n";
		std::cout << "nPoints: " << nPoints << "\n";
		getchar();
	}
	else
	{
		if (debug)
		{
			std::cout << "nPoints: " << nPoints << "\n";
			std::cout << "time \tticks \tv \n";
		}

		for (U32 ii = 0; ii < nPoints; ii++)
		{
			double V = Vi + (Vf - Vi)*ii / (nPoints - 1);
			queue.push(AnalogOut(dt, V));

			if (debug)
				std::cout << (ii + 1) * dt << "\t" << (ii + 1) * us2tick(dt) << "\t" << V << "\t" << "\n";
		}

		if (debug)
			getchar();

	}
	return queue;
}


void CountPhotons(NiFpga_Status* status, NiFpga_Session session) {

	size_t Npop = (Npixels + 1)* Nmaxlines;
	uint32_t r; //elements remaining
	size_t timeout = 100;
	uint8_t* data = new uint8_t[Npop];
	for (U32 ii = 0; ii < Npop; ii++)
		data[ii] = -1;

	//Start the host FIFO. Not needed for reading the data, but it takes about 3ms to read 'elementsRemaining' once the FIFO starts running.
	NiFpga_MergeStatus(status, NiFpga_StartFifo(session, NiFpga_FPGA_TargetToHostFifoU8_FIFOOUT));

	// read the DMA FIFO data and print. This function alone is able to start the FIFO, but it would not read 'elementsRemaining' right away because it takes about 3ms to read 'elementsRemaining' once the FIFO starts running
	NiFpga_MergeStatus(status, NiFpga_ReadFifoU8(session, NiFpga_FPGA_TargetToHostFifoU8_FIFOOUT, data, Npop, timeout, &r));
	
	std::cout << "Data: " << (U16)data[0] << " (garbage count before the first pixel)\n";
	//print out the data
	for (U32 ii = 1; ii < Npop; ii++)
		std::cout << "Data: " << (U16)data[ii] << "\n";
	std::cout << "Number of elements remaining in host FIFO: " << r << "\n";

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