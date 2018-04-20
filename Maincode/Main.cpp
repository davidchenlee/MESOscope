//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"

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

			NiFpga_MergeStatus(&status, NiFpga_Close(session, 1));			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
																			//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
																			//0 resets, 1 does not reset
		}	

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << std::endl;
		
	}
	
	std::cout << "\nPress any key to continue..." << std::endl;
	getchar();

	return 0;
}