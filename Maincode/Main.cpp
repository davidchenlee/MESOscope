//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"

/*
int main()
{
	NiFpga_Status status = NiFpga_Initialize();								//Must be called before any other FPGA calls
	std::cout << "FPGA initialize status: " << status << std::endl;
	
	if (NiFpga_IsNotError(status))											//Check for any FPGA error
	{
		NiFpga_Session session;

		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &session));		//Opens a session, downloads the bitstream
																												//1=no run, 0=run
		std::cout << "FPGA open-session status: " << status << std::endl;

		if (NiFpga_IsNotError(status))
		{
			//NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));			//Run the FPGA application if the FPGA was opened in 'no-run' mode

			runCombinedSequence(&status, session);

			NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
																			//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
																			//0 resets, 1 does not reset
			std::cout << "FPGA closing-session status: " << status << std::endl;
		}	

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << std::endl;
		
	}


	
	std::cout << "\nPress any key to continue..." << std::endl;
	getchar();

	return 0;
}
*/

int main()
{

	FPGAapi fpga;

	//Initialize the FPGA
	fpga.initialize();

	//Linear ramp for the galvo
	const double galvoAmplitude_um = 200 * um;
	const double galvoAmplitude_volt = galvoAmplitude_um * Galvo_voltPerUm;
	const double galvoTimeStep_us = 8 * us;

	RTsequence sequences(&fpga);
	sequences.linearRamp(GALVO1, galvoTimeStep_us, 25 * ms, galvoAmplitude_volt, -galvoAmplitude_volt);

	fpga.loadRTsequenceOnFPGA();

	NiFpga_Status status = 0;
	//Execute the commands and read the photon count
	readPhotonCount(&status, fpga.getSession());

	triggerFIFOflush(fpga.getSession());

	
	getchar();

	return 0;
}