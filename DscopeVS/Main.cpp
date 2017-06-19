//#include <stdio.h>
//#include <stdlib.h> //For malloc
//#include <ansi_c.h>
//#include <limits.h>
//#include <math.h>
//#include <time.h>
#include "windows.h"		//the stages use this lib. also Sleep
#include "NiFpga_FPGA.h"
#include <iostream>
#include "main.h"
//#include <concrt.h> 	//Concurrency::wait(2000);



/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;


void linearRamp(uint32_t *aa, tt_t ti, int16_t Vi, tt_t tf, int16_t Vf)
{
	int nPoints = (int) ((1.*tf-ti) / AO_dt);
	
	tt_t *tPoints = new tt_t[nPoints + 1];
	//uint16_t *vPoints = new uint16_t[nPoints+1];
	for (int ii = 0; ii < nPoints; ii++)
	{
		tPoints[ii] = ti + ii * AO_dt;
		//vPoints[ii] = VOUT( Vi + (1.*Vf-Vi)/(tf-ti)*(tPoints[ii] - ti)   );

		aa[ii] = AO_dt << Abits | (LSBmask & (  VOUT(  Vi + (1.*Vf - Vi) / (tf - ti)*(tPoints[ii] - ti)  )  )  );
	}
	//tPoints[nPoints] = tf;
	//vPoints[nPoints] = VOUT(Vf);
	aa[nPoints] = AO_dt << Abits | (LSBmask & VOUT(Vf));

	
	//std::cout << nPoints;
	/*
	for (int ii = 0; ii <= nPoints; ii++)
		std::cout << tPoints[ii] << "\n";
	getchar();
	*/
	
}





int main()
{
	/* must be called before any other FPGA calls */
	NiFpga_Status status = NiFpga_Initialize();

	std::cout << "FPGA status: " << status << "\n";

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


			const size_t sizeFifo = 23+1;

			/*AO1 FIFO*/
			size_t rAO1; //empty elements remaining
			uint32_t *AOfifo = new uint32_t[sizeFifo];
			for (int i = 0; i < sizeFifo; i++) { //initialize the array
				AOfifo[i] = 0;
			}

			tt_t t0 = 0*us;
			int16_t Vout0 = VOUT(10);
			tt_t t1 = 160*us;
			int16_t Vout1 = VOUT(0);

			//AOfifo[0] = (t0 << Abits) | (LSBmask & Vout0);
			//AOfifo[1] = (t1 << Abits) | (LSBmask & Vout1);
			//AOfifo[2] = 0x00A03FFF;
			//AOfifo[3] = 0x00A00000;
			//AOfifo[4] = 0x00A00000;
			//AOfifo[5] = 0x00A00000;
			linearRamp(AOfifo, 0*us, 0, 1150*us, 10);
			
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_A0FIFO, AOfifo, sizeFifo, timeout, &rAO1)); //send the AO data


			/*DO FIFO*/
			size_t rDO1; //empty elements remaining
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
			NiFpga_MergeStatus(&status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_DOFIFO, DOfifo, sizeFifo, timeout, &rDO1)); //send the DO data

			/* run the FPGA application.*/
			NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

			/*trigger the FPGA*/
			NiFpga_Bool start = 1;
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_start, start));


			/* close the session. THIS TURNS OFF THE OUTPUT OF THE FPGA */
			Sleep(2000);//temp hack to let the FPGA finish before shutting it down
		
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
int16_t test = 160;
printf("%i\n", test);
char hex[16];
sprintf(hex, "%x", ((test + (1 << Abits)) % (1 << Abits)));
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