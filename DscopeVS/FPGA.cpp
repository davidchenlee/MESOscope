#include "FPGA.h"
#include <limits.h>		//for _I16_MAX??
#include <iostream>
#include "windows.h"	//the stages use this lib. also Sleep
#include <queue>
using namespace std;

void printHex(int16_t input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

std::queue<uint32_t> linearRamp(tt_t ti, tt_t tf, double Vi, double Vf)
{
	std::queue<uint32_t> OUTqueue;

	tt_t dt = AO_dt;							//Analog output time increament in us
	int nPoints = (int)((1.*tf - ti) / dt);		//number of points

	tt_t *tPoints = new tt_t[nPoints];			//time points in us
	double *vPoints = new double[nPoints];		//voltage points in V

	if (nPoints <= 1)
	{
		std::cout << "ERROR: not enought points for the analog linear ramp\n";
		std::cout << "nPoints: " << nPoints << "\n";
	}
	else
	{
		for (int ii = 0; ii < nPoints; ii++)
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
		for (int ii = 0; ii < nPoints; ii++)
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

	std::cout << "FPGA status: " << status << "\n";

	/* check for any FPGA error */
	if (NiFpga_IsNotError(status))
	{
		NiFpga_Session session;

		/* opens a session, downloads the bitstream, but does not run the FPGA */
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", 0, &session)); //1=no run, 0=run

		if (NiFpga_IsNotError(status))
		{
			//Initialize the FPGA
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
			NiFpga_MergeStatus(&status, NiFpga_WriteI32(session, NiFpga_FPGA_ControlI32_FIFOtimeout, 1000));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOdelaytick, DODelayTick));//DELAY. Sync AO and DO by delaying DO

			//run the FPGA application
			//NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));


			std::queue<uint32_t> FIFOAO1, FIFOAO2, FIFODO1;

			//AO1
			tt_t At0 = us2tick(4 * us);//40 tick = 1 us
			tt_t At1 = us2tick(1400 * us);
			tt_t At2 = us2tick(4 * us);
			tt_t At3 = us2tick(4 * us);
			int16_t VO0 = AOUT(10);
			int16_t VO1 = AOUT(0);
			int16_t VO2 = AOUT(0);
			int16_t VO3 = AOUT(0);

			FIFOAO1.push(u32pack(At0, VO0));
			FIFOAO1.push(u32pack(At1, VO1));
			FIFOAO1.push(u32pack(At2, VO2));
			FIFOAO1.push(u32pack(At3, VO3));

			//AO2
			FIFOAO2.push(u32pack(us2tick(5 * us), VO0));
			FIFOAO2.push(u32pack(At1, VO1));
			FIFOAO2.push(u32pack(At2, VO2));
			FIFOAO2.push(u32pack(At3, VO3));

			//DO1
			tt_t Dt0 = us2tick(4 * us); //40 tick = 1 us
			tt_t Dt1 = us2tick(1000 * us);
			tt_t Dt2 = us2tick(4 * us);
			tt_t Dt3 = us2tick(4 * us);
	
			FIFODO1.push(u32pack(Dt0, 0x0001));
			FIFODO1.push(u32pack(Dt1, 0x0000));
			FIFODO1.push(u32pack(Dt2, 0x0001));
			FIFODO1.push(u32pack(Dt3, 0x0000));




			//linear output
			std::queue<uint32_t> linearR = linearRamp(0 * us, 16 * us, 0, 10);
			linearR.push(u32pack(4*us, 0));


			//Merge the queues
			std::queue<uint32_t> FIFOqueue;
			//AO1
			FIFOqueue.push(FIFOAO1.size()); //push the number of elements in the queue
			while (!FIFOAO1.empty())
			{
				FIFOqueue.push(FIFOAO1.front());
				FIFOAO1.pop();
			}
			//AO2
			FIFOqueue.push(FIFOAO2.size()); //push the number of elements in the queue
			while (!FIFOAO2.empty())
			{
				FIFOqueue.push(FIFOAO2.front());
				FIFOAO2.pop();
			}
			//DO1
			FIFOqueue.push(FIFODO1.size()); //push the number of elements in the queue
			while (!FIFODO1.empty())
			{
				FIFOqueue.push(FIFODO1.front());
				FIFODO1.pop();
			}
			FIFOAO1, FIFOAO2, FIFODO1 = {}; //cleanup



			//transfer the queue to an array. THE ORDER DETERMINES THE TARGETED CHANNEL
			size_t sizeFIFOqueue = FIFOqueue.size();
			uint32_t *FIFOarray = new uint32_t[sizeFIFOqueue];
			for (int i = 0; i < sizeFIFOqueue; i++)
			{
				FIFOarray[i] = FIFOqueue.front();
				FIFOqueue.pop();
			}
			FIFOqueue = {}; //cleanup



			//send the data to the FPGA through the FIFO
			uint32_t timeout = -1; // in ms. -1 means no timeout
			size_t r; //empty elements remaining
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFOarray, sizeFIFOqueue, timeout, &r));
			PulseTrigger(&status, session);
			delete[] FIFOarray;


			//SECOND ROUND
			if (1)
			{
				tt_t At0 = us2tick(4 * us);//40 tick = 1 us
				tt_t At1 = us2tick(4 * us);
				int16_t Vout0 = AOUT(5);
				int16_t Vout1 = AOUT(0);

				//AO1
				FIFOAO1.push(u32pack(At0, VO0));
				FIFOAO1.push(u32pack(At1, VO1));
				//AO2
				FIFOAO2.push(u32pack(At0, VO0));
				FIFOAO2.push(u32pack(At1, VO1));
				//DO1
				FIFODO1.push(u32pack(Dt0, 0x0001));
				FIFODO1.push(u32pack(Dt1, 0x0000));



				//Merge the queues
				std::queue<uint32_t> FIFOqueue;
				//AO1
				FIFOqueue.push(FIFOAO1.size()); //push the number of elements in the queue
				while (!FIFOAO1.empty())
				{
					FIFOqueue.push(FIFOAO1.front());
					FIFOAO1.pop();
				}
				//AO2
				FIFOqueue.push(FIFOAO2.size()); //push the number of elements in the queue
				while (!FIFOAO2.empty())
				{
					FIFOqueue.push(FIFOAO2.front());
					FIFOAO2.pop();
				}
				//DO1
				FIFOqueue.push(FIFODO1.size()); //push the number of elements in the queue
				while (!FIFODO1.empty())
				{
					FIFOqueue.push(FIFODO1.front());
					FIFODO1.pop();
				}
				FIFOAO1, FIFOAO2, FIFODO1 = {}; //cleanup

				
				//transfer the queue to an array. THE ORDER DETERMINES THE TARGETED CHANNEL
				size_t sizeFIFOqueue = FIFOqueue.size();
				uint32_t *FIFOarray = new uint32_t[sizeFIFOqueue];
				for (int i = 0; i < sizeFIFOqueue; i++)
				{
					FIFOarray[i] = FIFOqueue.front();
					FIFOqueue.pop();
				}
				FIFOqueue = {}; //cleanup


				//send the data through FIFO
				NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFOarray, sizeFIFOqueue, timeout, &r));
				PulseTrigger(&status, session);
				delete[] FIFOarray;
				PulseTrigger(&status, session);
			}


			//EVIL FUNCTION. DO NOT USE
			/* Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target)
			unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.*/
			//NiFpga_MergeStatus(&status, NiFpga_Close(session, 1)); //0 resets, 1 does not reset


		}


		/* You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.*/
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
	}
}

//converts microseconds to ticks
tt_t us2tick(double x)
{
	return (tt_t)(x * tickPerUs);
}

//converts voltage (range: -10 to 10) to signed int 16 (range: -32768 to 32767)
//0x7FFFF = 0d32767
//0xFFFF = -1
//0x8000 = -32768
int16_t AOUT(double x)
{
	return (int16_t)(x / 10 * _I16_MAX);
}

//pack t as MSB and x as LSB
uint32_t u32pack(tt_t t, uint16_t x)
{
	return (t << Abits) | (LSBmask & x);
}


void wait(tt_t gt, tt_t dt)
{
	gt += dt;
}

void DOUT(int channel, int bit)
{

}

void PulseTrigger(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
}

void PulseStart(NiFpga_Status* status, NiFpga_Session session)
{
	//NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start, 1));
	//NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start, 0));
}


std::queue<uint32_t> MergeQueues(std::queue<uint32_t> FIFOAO1, std::queue<uint32_t> FIFOAO2, std::queue<uint32_t> FIFODO1)
{
	//create the FIFO queue
	std::queue<uint32_t> FIFOqueue;
	//AO1
	FIFOqueue.push((FIFOAO1).size()); //push the number of elements in the queue
	while (!FIFOAO1.empty())
	{
		FIFOqueue.push(FIFOAO1.front());
		FIFOAO1.pop();
	}
	//AO2
	FIFOqueue.push(FIFOAO2.size()); //push the number of elements in the queue
	while (!FIFOAO2.empty())
	{
		FIFOqueue.push(FIFOAO2.front());
		FIFOAO2.pop();
	}
	//DO1
	FIFOqueue.push(FIFODO1.size()); //push the number of elements in the queue
	while (!FIFODO1.empty())
	{
		FIFOqueue.push(FIFODO1.front());
		FIFODO1.pop();
	}
	return FIFOqueue;
}



/*for the digital channel, I will define the sequece as
wait(t1)
DOUT(channel, bit)
wait(t2)
DOUT(channel, bit)
...etc

I would like to convert it into the form

DOUT(channel, duration, bit,)
DOUT(channel, duration, bit,)
...etc


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