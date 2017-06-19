//#include <stdio.h>
//#include <stdlib.h> //For malloc
//#include <ansi_c.h>
//#include <limits.h>
//#include <math.h>
#include <time.h>
#include "NiFpga_FPGA.h"
#include <iostream>
#include "main.h"



/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;



/*Halt the program (in seconds)*/
void delay(double dly) {
	/* save start time */
	const time_t start = time(NULL);
	
	time_t current;
	do {
		/* get current time */
		time(&current);

		/* break loop when the requested number of seconds have elapsed */
	} while (difftime(current, start) < dly);
}


int main()
{
	/* must be called before any other FPGA calls */
	NiFpga_Status status = NiFpga_Initialize();

	std::cout << "FPGA status: " << status;

	/* check for any FPGA error */
	if (NiFpga_IsNotError(status))
	{
		NiFpga_Session session;

		/* opens a session, downloads the bitstream, but does not run the FPGA */
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", NiFpga_OpenAttribute_NoRun, &session));

		if (NiFpga_IsNotError(status))
		{
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_InitialWaittick, initialWait));

			/*DELAY. Sync AO and DO by delaying DO*/
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOFIFODelaytick, DOfifoDelayTick));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_AOCalibratetick, calibrateAOtiming));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOCalibratetick, calibrateDOtiming));


			const size_t sizeFifo = 2;
			/*AO1 FIFO*/
			size_t r1; //empty elements remaining
			uint32_t *AOfifo = new uint32_t[sizeFifo];
			for (int i = 0; i < sizeFifo; i++) { //initialize the array
				AOfifo[i] = 0;
			}

			int16_t t0 = 0x0000;
			int16_t Vout0 = VOUT(10);
			int16_t t1 = 0x00A0;
			int16_t Vout1 = VOUT(0);

			AOfifo[0] = (t0 << Abits) | (LSBmask & Vout0);
			AOfifo[1] = (t1 << Abits) | (LSBmask & Vout1);
			//AOfifo[2] = 0x00A03FFF;
			//AOfifo[3] = 0x00A00000;
			//AOfifo[4] = 0x00A00000;
			//AOfifo[5] = 0x00A00000;
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_A0FIFO, AOfifo, sizeFifo, timeout, &r1)); //send the AO data

																																			   /*DO FIFO*/
			size_t r2; //empty elements remaining
			uint32_t *DOfifo = new uint32_t[sizeFifo];

			for (int i = 0; i < sizeFifo; i++) { //initialize the array
				DOfifo[i] = 0;
			}
			DOfifo[0] = 0x00000001;
			DOfifo[1] = 0x00A00000;
			//DOfifo[2] = 0x00A00001;
			//DOfifo[3] = 0x00A00001;
			//DOfifo[4] = 0x00A00000;
			//DOfifo[5] = 0x00A00000;
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_DOFIFO, DOfifo, sizeFifo, timeout, &r2)); //send the DO data


			/* run the FPGA application.*/
			NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

			/*trigger the FPGA*/
			NiFpga_Bool start = 1;
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_start, start));



			/* close the session. THIS TURNS OFF THE OUTPUT OF THE FPGA */
			delay(2); //temp hack to let the FPGA finish before shutting it down
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

			/*cleanup*/
			delete[] AOfifo;
			delete[] DOfifo;

		}


		/* must be called after all other calls */
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
	}



	return 0;
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
int16_t ahex = (int16_t) (10.0/10*_I16_MAX);
printf("%i\n", ahex);
char hex[16];
sprintf(hex, "%x", ((ahex + (1 << Abits)) % (1 << Abits)));
puts(hex);
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