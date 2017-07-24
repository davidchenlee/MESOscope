#include "FPGA.h"

void printHex(int16_t input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

U32Q linearRamp(tt_t dt, tt_t ti, tt_t tf, double Vi, double Vf)
{
	U32Q queue;		
	bool debug = false;
	
	if (dt < AO_dt)
	{
		std::cout << "WARNING: step size too small. Step size set to " << AO_dt << " us";
		dt = AO_dt; //Analog output time increment in us
		getchar();
	}

	uint32_t nPoints = (uint32_t)((tf - ti) / dt);		//number of points

	if (nPoints <= 1)
	{
		std::cout << "ERROR: not enought points for the analog linear ramp\n";
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

		for (uint32_t ii = 0; ii < nPoints; ii++)
		{
			tt_t tPoint = ti + ii * dt; 
			double vPoint = Vi + (Vf - Vi)*ii / (nPoints - 1);
			queue.push( u32pack(dt*tickPerUs, AOUT(vPoint)) );
			
			if (debug)
				std::cout << tPoint << "\t" << ii * dt*tickPerUs << "\t" << vPoint << "\t" << "\n";
		}

		if (debug)
			getchar();

	}
	return queue;
}


//converts microseconds to ticks
tt_t us2tick(double x)
{
	return (tt_t)(x * tickPerUs);
}

//converts voltage (range: -10V to 10V) to signed int 16 (range: -32768 to 32767)
//0x7FFFF = 0d32767
//0xFFFF = -1
//0x8000 = -32768
int16_t AOUT(double x)
{
	return (int16_t)(x / 10*_V * _I16_MAX);
}

//pack t as MSB and x as LSB
uint32_t u32pack(tt_t t, uint16_t x)
{
	return (t << Abits) | (LSBmask & x);
}

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteI32(session, NiFpga_FPGA_ControlI32_FIFOtimeout, 80));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOdelaytick, DODelayTick));//DELAY. Sync AO and DO by delaying DO
	std::cout << "FPGA initialize-variables status: " << *status << "\n";
}


void PulseTrigger(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	std::cout << "Pulse trigger: " << *status << "\n";
}


void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& QV)
{
	//take a vector of queues and return it as a single queue
	U32Q allQs;
	for (uint32_t i = 0; i < Nchannels; i++)
	{
		allQs.push(QV[i].size()); //push the number of elements in the queue
		while (!QV[i].empty())
		{
			allQs.push(QV[i].front());
			QV[i].pop();
		}
	}
	//transfer the queue to an array. THE ORDER DETERMINES THE TARGETED CHANNEL
	uint32_t sizeFIFOqueue = allQs.size();
	uint32_t* FIFO = new uint32_t[sizeFIFOqueue];
	for (uint32_t i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQs.front();
		allQs.pop();
	}
	allQs = {};//cleanup

	//send the data to the FPGA through the FIFO
	uint32_t timeout = -1; // in ms. -1 means no timeout
	uint32_t r; //empty elements remaining
	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << "\n";
	delete[] FIFO;//cleanup
}

// ADD the 'tailQ' queue to the end of the 'headQ' queue
U32Q ConcatenateQ(U32Q& headQ, U32Q& tailQ)
{
	while (!tailQ.empty())
	{
		headQ.push(tailQ.front());
		tailQ.pop();
	}
	return headQ;
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