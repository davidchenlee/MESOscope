#include "FPGA.h"
#include <limits.h>		//for _I16_MAX??
#include <iostream>
#include "windows.h"	//the stages use this lib. also Sleep
//using namespace std;

void printHex(int16_t input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

std::queue<uint32_t> linearRamp(tt_t dt, tt_t ti, tt_t tf, double Vi, double Vf)
{
	std::queue<uint32_t> OUTqueue;					
	
	if (dt < AO_dt)
	{
		std::cout << "WARNING: step size too small. Step size set to " << AO_dt << " us";
		dt = AO_dt; //Analog output time increment in us
		getchar();
	}

	uint32_t nPoints = (uint32_t)((tf - ti) / dt);		//number of points

	tt_t* tPoints = new tt_t[nPoints];			//time (in us)
	double *vPoints = new double[nPoints];		//voltage (in V)

	if (nPoints <= 1)
	{
		std::cout << "ERROR: not enought points for the analog linear ramp\n";
		std::cout << "nPoints: " << nPoints << "\n";
		getchar();
	}
	else
	{
		for (uint32_t ii = 0; ii < nPoints; ii++)
		{
			tPoints[ii] = ti + ii * dt; //just for debugging 
			vPoints[ii] = Vi + (Vf - Vi)*ii / (nPoints - 1);
			OUTqueue.push( u32pack(dt*tickPerUs, AOUT(vPoints[ii])) );
		}
	}

	//for debugging
	if (0)
	{
		std::cout << "nPoints: " << nPoints << "\n";

		std::cout << "time \tticks \tv \n";
		for (uint32_t ii = 0; ii < nPoints; ii++)
		{
			std::cout << tPoints[ii] << "\t" << ii * dt*tickPerUs << "\t" << vPoints[ii] << "\t" << "\n";
			//std::cout << ii << "\t" << aa[ii] << "\n";
		}
		getchar();
	}
	delete[] tPoints, vPoints;
	return OUTqueue;
}


void RunFPGA()
{
	/* must be called before any other FPGA calls */
	NiFpga_Status status = NiFpga_Initialize();
	std::cout << "FPGA initialize status: " << status << "\n";

	/* check for any FPGA error */
	if (NiFpga_IsNotError(status))
	{
		NiFpga_Session session;

		/* opens a session, downloads the bitstream*/
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", 0, &session)); //1=no run, 0=run
		std::cout << "FPGA open-session status: " << status << "\n";

		if (NiFpga_IsNotError(status))
		{
			InitializeFPGA(&status, session);

			//run the FPGA application if the FPGA was opened in 'no-run' mode
			//NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

			//Create an array of queues. Assigns a queue at each channel
			std::queue<uint32_t>* Qarray = new std::queue<uint32_t>[Nchannels];

			//AO1
			tt_t At0 = us2tick(4 * _us);//40 tick = 1 us
			tt_t At1 = us2tick(1400 * _us);
			tt_t At2 = us2tick(4 * _us);
			tt_t At3 = us2tick(4 * _us);
			int16_t VO0 = AOUT(10*_V);
			int16_t VO1 = AOUT(0*_V);
			int16_t VO2 = AOUT(0*_V);
			int16_t VO3 = AOUT(0*_V);
			Qarray[0].push(u32pack(At0, VO0));
			Qarray[0].push(u32pack(At1, VO1));
			Qarray[0].push(u32pack(At2, VO2));
			Qarray[0].push(u32pack(At3, VO3));

			//AO2
			Qarray[1].push(u32pack(us2tick(5 * _us), VO0));
			Qarray[1].push(u32pack(At1, VO1));
			Qarray[1].push(u32pack(At2, VO2));
			Qarray[1].push(u32pack(At3, VO3));

			//DO1
			tt_t Dt0 = us2tick(4 * _us); //40 tick = 1 us
			tt_t Dt1 = us2tick(1 * _ms);
			tt_t Dt2 = us2tick(4 * _us);
			tt_t Dt3 = us2tick(4 * _us);
			Qarray[2].push(u32pack(Dt0, 0x0001));
			Qarray[2].push(u32pack(Dt1, 0x0000));
			Qarray[2].push(u32pack(Dt2, 0x0001));
			Qarray[2].push(u32pack(Dt3, 0x0000));


			//linear output
			std::queue<uint32_t> linearR = linearRamp(4*_us, 0 * _us, 1 * _ms, 0, 10*_V);
			linearR.push(u32pack(4*_us, 0));//return to zero
			//Qarray[0] = linearR;//overwrites FIFO[0] with a linear ramp


			SendOutQueue(&status, session, Qarray);
			PulseTrigger(&status, session);
			delete[] Qarray;//cleanup
			
	
			//SECOND ROUND
			if (1)
			{
				std::queue<uint32_t>* Qarray2 = new std::queue<uint32_t>[Nchannels];

				tt_t At0 = us2tick(4 * _us);//40 tick = 1 us
				tt_t At1 = us2tick(4 * _us);
				int16_t VO0 = AOUT(5*_V);
				int16_t VO1 = AOUT(0);

				//AO1
				Qarray2[0].push(u32pack(At0, VO0));
				Qarray2[0].push(u32pack(At1, VO1));
				//AO2
				Qarray2[1].push(u32pack(At0, VO0));
				Qarray2[1].push(u32pack(At1, VO1));
				//DO1
				Qarray2[2].push(u32pack(Dt0, 0x0001));
				Qarray2[2].push(u32pack(Dt1, 0x0000));

				SendOutQueue(&status, session, Qarray2);
				PulseTrigger(&status, session);

				delete[] Qarray2;//cleanup
			}


			//EVIL FUNCTION. DO NOT USE
			/* Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target)
			unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.*/
			//NiFpga_MergeStatus(&status, NiFpga_Close(session, 1)); //0 resets, 1 does not reset


		}


		/* You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.*/
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << "\n";
		Sleep(2000);
	}
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


void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, std::queue<uint32_t>* Qarray)
{
	//take an array of queues and return them as a single queue
	std::queue<uint32_t> allQs;
	for (size_t i = 0; i < Nchannels; i++)
	{
		allQs.push(Qarray[i].size()); //push the number of elements in the queue
		while (!Qarray[i].empty())
		{
			allQs.push(Qarray[i].front());
			Qarray[i].pop();
		}
	}
	//transfer the queue to an array. THE ORDER DETERMINES THE TARGETED CHANNEL
	size_t sizeFIFOqueue = allQs.size();
	uint32_t* FIFO = new uint32_t[sizeFIFOqueue];
	for (size_t i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQs.front();
		allQs.pop();
	}
	allQs = {};//cleanup

	//send the data to the FPGA through the FIFO
	uint32_t timeout = -1; // in ms. -1 means no timeout
	size_t r; //empty elements remaining
	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << "\n";
	delete[] FIFO;//cleanup
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