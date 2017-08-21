//#include <math.h>
//#include <time.h>
//#include <concrt.h> 	//Concurrency::wait(2000);
#include "windows.h"	//the stages use this lib. also Sleep
#include "FPGA.h"
#include "Sequences.h"
#include "Seq.h"
//using namespace std;


int main()
{
	if (0)
	{
		Seq ss;
		ss.shutter(1 * us, 1);
		std::cout << "size of the vector" << ss.size() << "\n";
		std::cout << "" << (ss.vector())[0].size() << "\n";
		Sleep(1000);
	}
	else {
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

			SendOutQueue(&status, session, Seq1());
			PulseTrigger(&status, session);

			Sleep(500);

			// start acquiring data
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Startacquisition, 1));
			Sleep(500);
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Readdata, 1));
			Sleep(500);
			
			size_t Npop = (Nmaxpixels+1)* Nmaxlines;
			uint32_t r; //elements remaining
			size_t timeout = 100;
			uint16_t* data = new uint16_t[Npop];
			for (int ii = 0; ii < Npop; ii++)
				data[ii] = -1;


			//Start up the host FIFO. No need for reading the data, but it takes about 3ms to read 'elementsRemaining' once the FIFO starts running.
			NiFpga_MergeStatus(&status, NiFpga_StartFifo(session, NiFpga_FPGA_TargetToHostFifoU16_FIFOcounters));
			Sleep(10);

			// read the DMA FIFO data and print. This function alone is able to start up the FIFO, but it would not read 'elementsRemaining' right away because it takes about 3ms to read 'elementsRemaining' once the FIFO starts running
			NiFpga_MergeStatus(&status, NiFpga_ReadFifoU16(session, NiFpga_FPGA_TargetToHostFifoU16_FIFOcounters, data, Npop, timeout, &r));
			for (int ii = 0; ii<Npop; ii++)
				std::cout << "Data: " << data[ii] << "\n";
			std::cout << "Number of elements remaining in host FIFO: " << r << "\n";
			
			
			
			
			
			Sleep(100);


			//NiFpga_StopFifo(session, NiFpga_FPGA_TargetToHostFifoU16_FIFOcounters)



			//SECOND ROUND
			if (0)
			{
				SendOutQueue(&status, session, Seq2());
				PulseTrigger(&status, session);
			}

			/* Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target)
			unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.*/
			//NiFpga_MergeStatus(&status, NiFpga_Close(session, 0)); //0 resets, 1 does not reset
		}

		//Reset the FPGA
		//NiFpga_Reset(session);


		/* You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.*/
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << "\n";
		
		getchar();
	}
	}


	return 0;
}
