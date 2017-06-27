#include "FPGA.h"
#include <limits.h>		//for _I16_MAX??
#include <iostream>
#include "windows.h"	//the stages use this lib. also Sleep
#include <queue>

void printHex(int16_t input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

void linearRamp(uint32_t *aa, tt_t ti, double Vi, tt_t tf, double Vf)
{
	tt_t dt = AO_dt;  //Analog output time increament in us
	int nPoints = (int)((1.*tf - ti) / dt); //number of points

	tt_t *tPoints = new tt_t[nPoints]; //time points in us
	double *vPoints = new double[nPoints]; //voltage points in V

	if (nPoints <= 1)
	{
		aa[0] = 0;
		std::cout << "ERROR: not enought points for the analog linear ramp\n";
		std::cout << "nPoints: " << nPoints << "\n";
	}
	else
	{
		for (int ii = 0; ii < nPoints; ii++)
		{
			tPoints[ii] = ti + ii * dt; //just for debugging 
			vPoints[ii] = Vi + (Vf - Vi)*ii / (nPoints - 1);
			aa[ii] = u32pack(dt*tickPerUs, AOUT(vPoints[ii]) );
		}
		delete[] tPoints;
		delete[] vPoints;
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
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", 1, &session));

		if (NiFpga_IsNotError(status))
		{

			
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Retrigger, 0));

			/*DELAY. Sync AO and DO by delaying DO*/
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOdelaytick, DODelayTick));

			tt_t ti = 0 * us;
			tt_t tf = 1400 * us;
			double vi = 0;
			double vf = 10;
			int nPoints = (int)((1.*tf - ti) / AO_dt);

			//FIFO
			uint32_t timeout = 1; // in ms. 
			size_t r; //empty elements remaining
			//uint32_t sizeFifo = nPoints;
			uint32_t sizeFifo = 12;
			uint32_t *FIFO = new uint32_t[sizeFifo];
			for (uint32_t i = 0; i < sizeFifo; i++) { //initialize the array
				FIFO[i] = 0;
			}


			if (1)
			{
				//send out the size of the AO FIFO
				NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NAO1, 4));
				NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NAO2, 4));
				NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NDO1, 4));

				tt_t At0 = us2tick(4*us);//40 tick = 1 us
				tt_t At1 = us2tick(1400*us);
				tt_t At2 = us2tick(4*us);
				tt_t At3 = us2tick(4*us);
				int16_t Vout0 = AOUT(10);
				int16_t Vout1 = AOUT(0);
				int16_t Vout2 = AOUT(0);
				int16_t Vout3 = AOUT(10);

				//AO1
				FIFO[0] = u32pack(At0,Vout0);
				FIFO[1] = u32pack(At1, Vout1);
				FIFO[2] = u32pack(At2, Vout2);
				FIFO[3] = u32pack(At3, Vout3);
				//AO2
				FIFO[4] = u32pack(us2tick(5 * us), Vout0);
				FIFO[5] = u32pack(At1, Vout1);
				FIFO[6] = u32pack(At2, Vout2);
				FIFO[7] = u32pack(At3, Vout3);

				//DO1
				tt_t Dt0 = us2tick(4.0 * us); //40 tick = 1 us
				tt_t Dt1 = us2tick(1400 * us);
				tt_t Dt2 = us2tick(4 * us);
				tt_t Dt3 = us2tick(4 * us);
				FIFO[8] = u32pack(Dt0, 0x0001);
				FIFO[9] = u32pack(Dt1, 0x0000);
				FIFO[10] = u32pack(Dt3, 0x0000);
				FIFO[11] = u32pack(Dt3, 0x0001);
			}
			else {
				NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NAO1, sizeFifo));
				linearRamp(FIFO, ti, vi, tf, vf);
			}
			//send the data through FIFO
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFO, sizeFifo, timeout, &r));


			/* run the FPGA application.*/
			NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));


			/*trigger the FPGA*/
			NiFpga_Bool start = 1;
			Sleep(10);
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_start, start));

			Sleep(1);
			
			/*
			//SECOND ROUND
			//send out the size of the AO FIFO
			NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NAO1, 2));
			NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NAO2, 2));
			NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_NDO1, 2));
			tt_t At0 = us2tick(4 * us);//40 tick = 1 us
			tt_t At1 = us2tick(4 * us);
			int16_t Vout0 = AOUT(10);
			int16_t Vout1 = AOUT(0);

			//AO1
			FIFO[0] = u32pack(At0, Vout0);
			FIFO[1] = u32pack(At1, Vout1);
			//AO2
			FIFO[2] = u32pack(At0, Vout0);
			FIFO[3] = u32pack(At1, Vout1);
			//DO1
			FIFO[4] = u32pack(At0, 0x0000);
			FIFO[5] = u32pack(At1, 0x0000);

			
			//send the data through FIFO and re-trigger
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFO, FIFO, sizeFifo, timeout, &r));
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Retrigger, 1));
			*/
			

			/* Closes the session to the FPGA.The FPGA resets (Re-downloads the FPGA bitstream to the target)
			unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.*/
			
			Sleep(2000);//Let the FPGA finish before shutting it down
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

			/*cleanup*/
			delete[] FIFO;
		}


		/* You must call this function after all other function calls if
		NiFpga_Initialize succeeds.This function unloads the NiFpga library.*/
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
	}
}

//converts microseconds to ticks
tt_t us2tick(double x)
{
	return (tt_t)(x * tickPerUs);
}

//converts voltage (range: -10 to 10) to signed int 16 (range: -32768 to 32767)
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
	gt += dt ;
}

void DOUT(int channel, int bit)
{

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