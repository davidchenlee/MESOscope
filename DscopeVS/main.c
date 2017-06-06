/*
*Date: April 2017
*Author: David Chen
*Institution: Max Planck Institute for Cell Biology and Genetics
*Description: Main program for controlling the DEEPscope via the NI 7852R card
*Source: http://www.ni.com/tutorial/8638/en/ 
*/


/*
2^16 * 25ns = 1.6384 ms
2^32 * 25ns = 107.3741824 s

maybe I shoud do he following package:
line 1: 16 bits for the channel ID, 16 bits for the AO value
line 2: 32 bits for the wait-time
*/


#include <stdio.h>
#include <stdlib.h> //For malloc
#include <math.h>
#include <time.h>
//#include <ansi_c.h>
//#include <limits.h>
#include "NiFpga_FPGA.h"
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



int main(void)
{
	/* must be called before any other FPGA calls */
	NiFpga_Status status = NiFpga_Initialize();

	printf("FPGA status: %d\n", status);


	/* check for any FPGA error */
	if (NiFpga_IsNotError(status))
	{
		NiFpga_Session session;

		/* opens a session, downloads the bitstream, but does not run the FPGA */
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", NiFpga_OpenAttribute_NoRun, &session));

		if (NiFpga_IsNotError(status))
		{
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

		


			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_InitialWaittick, initialWait));

			/*DELAY. Sync AO and DO by delaying DO*/
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOFIFODelaytick, DOfifoDelayTick));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_AOCalibratetick, calibrateAOtiming));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOCalibratetick, calibrateDOtiming));


			const size_t sizeFifo = 2;
			/*AO1 FIFO*/
			size_t r1; //empty elements remaining
			int32_t *AOfifo = malloc(sizeFifo * sizeof(int32_t));
			for (int i = 0; i < sizeFifo; i++) { //initialize the array
				AOfifo[i] = 0;
			}

			int16_t t0 = 0x0000;
			int16_t Vout0 = VOUT(-10);
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
			uint32_t *DOfifo = malloc(sizeFifo * sizeof(int32_t));

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
			delay(1); //temp hack to let the FPGA finish before shutting it down
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

			/*cleanup*/
			free(AOfifo);
			free(DOfifo);

		}


		/* must be called after all other calls */
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
	}



	return 0;
}



