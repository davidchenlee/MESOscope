#include "TCSequences.h"
#include "NiFpga_FPGAvi.h"
#include <iostream>


int main()
{
	//must be called before any other FPGA calls
	NiFpga_Status status = NiFpga_Initialize();
	std::cout << "FPGA initialize status: " << status << std::endl;

	//check for any FPGA error
	if (NiFpga_IsNotError(status))
	{
		NiFpga_Session session;

		//opens a session, downloads the bitstream
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &session)); //1=no run, 0=run
		std::cout << "FPGA open-session status: " << status << std::endl;

		if (NiFpga_IsNotError(status))
		{
			initializeFPGA(&status, session);

			//run the FPGA application if the FPGA was opened in 'no-run' mode
			//NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

			sendQueueToFPGA(&status, session, TestAODO());
			triggerFPGAreadsCommandsFromPC(&status, session);			//trigger the control sequence
			triggerFPGAstartsImaging(&status, session);

			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
			//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 1)); //0 resets, 1 does not reset
		}

		//Reset the FPGA
		//NiFpga_Reset(session);


		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << std::endl;

	}

	getchar();

	return 0;
}