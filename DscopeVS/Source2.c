#include <stdio.h>
//#include <ansi_c.h>
#include <limits.h>
#include "NiFpga_FPGA.h"

#define AOFIFODEPTH 3 //number of elements in the AO FIFO
#define DIOFIFODEPTH 3 //number of elements in the DIO FIFO

/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;

int main(void)
{
	/* must be called before any other FPGA calls */
	NiFpga_Status status = NiFpga_Initialize();

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


			/*AO1 FIFO*/
			uint32_t timeout = 10000/* 10 seconds */;




			uint32_t AOfifoRate = 160; //in tick=25ns. The min pulse length is 1us @tick=40. The measured pulse length is 1.15us 
			NiFpga_MergeStatus(&status, NiFpga_WriteU32(session, NiFpga_FPGA_ControlU32_AO1LoopPeriodus, AOfifoRate)); //rate



			size_t r1; //empty elements remaining
			size_t sizeAOfifo = AOFIFODEPTH;
			int16_t AOfifo[AOFIFODEPTH]; //the analog output takes a signed 16-bit int
			AOfifo[0] = _I16_MAX;
			AOfifo[1] = _I16_MIN;
			AOfifo[2] = _I16_MAX;
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoI16_A0FIFO, AOfifo, sizeAOfifo, timeout, &r1));


			/*DIO FIFO*/
			size_t r2; //empty elements remaining
			size_t sizeDIOfifo = DIOFIFODEPTH;
			NiFpga_Bool DIOfifo[DIOFIFODEPTH];
			DIOfifo[0] = 1;
			DIOfifo[1] = 0;
			DIOfifo[2] = 1;
		
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoBool(session, NiFpga_FPGA_HostToTargetFifoBool_DIOFIFO, DIOfifo, sizeDIOfifo, timeout, &r2));




			/* run the FPGA application.*/
			NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

			NiFpga_Bool start = 1;
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_start,start));



			/* close the session. THIS TURNS OFF THE OUTPUT OF THE FPGA */
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));
		}


		/* must be called after all other calls */
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
	}
	return 0;
}

