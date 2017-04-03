#include <stdio.h>
#include <stdlib.h> //For malloc
#include <time.h>
//#include <ansi_c.h>
//#include <limits.h>
#include "NiFpga_FPGA.h"


#define tickPerUs 40
#define us 1
#define tick 1

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
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0",
			NiFpga_OpenAttribute_NoRun, &session));

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

			/*		int16_t aa = 1;
			printf("%i\n", aa);
			getchar();*/



			/*FIFO variables*/
			uint32_t timeout = 1/* in ms */;
			//uint8_t fifoRateTick = 4 * us * tickPerUs; //a 40 MHZ clock means 1 tick=25ns. uint32_t DOfifoDelayTick = 40
			//NiFpga_MergeStatus(&status, NiFpga_WriteU8(session, NiFpga_FPGA_ControlU8_LoopPeriodtick, fifoRateTick)); //rate

			

			/*DELAY. Sync AO and DO by delaying DO*/
			uint8_t DOfifoDelayTick = 80 * tick; //relative delay between AO and DO
			uint16_t calibrateAOtiming = 40 * tick; //fine-tune the AO timing
			uint16_t calibrateDOtiming = 5 * tick; //fine-tune the DO timing
			NiFpga_MergeStatus(&status, NiFpga_WriteU8(session, NiFpga_FPGA_ControlU8_DOFIFODelayTicks, DOfifoDelayTick));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_AOCalibratetick, calibrateAOtiming));
			NiFpga_MergeStatus(&status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_DOCalibratetick, calibrateDOtiming));


			const size_t sizeFifo = 5;
			/*AO1 FIFO*/
			size_t r1; //empty elements remaining
			int32_t *AOfifo = malloc(sizeFifo * sizeof(int32_t));
			for (int i = 0; i < sizeFifo; i++) { //initialize the array
				AOfifo[i] = 0;
			}
			//AOfifo[0] = _I16_MAX;
			AOfifo[0] = 0x00007FFF;
			AOfifo[1] = 0x00A00000;
			AOfifo[2] = 0x00A03FFF;
			AOfifo[3] = 0x00A00000;
			AOfifo[4] = 0x00A00000;
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
			DOfifo[2] = 0x00A00001;
			DOfifo[3] = 0x00A00001;
			DOfifo[4] = 0x00A00000;
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



