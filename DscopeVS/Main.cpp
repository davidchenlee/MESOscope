//#include <math.h>
//#include <time.h>
//#include <concrt.h> 	//Concurrency::wait(2000);
#include "windows.h"	//the stages use this lib. also Sleep
#include "Paragraph.h"
//#include "Seq.h" //'Seq' class
//#include "PIstages.h"
//#include "UART.h"
#include "Tiffscope.h"


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
		NiFpga_MergeStatus(&status, NiFpga_Open(Bitfile, NiFpga_FPGA_Signature, "RIO0", 0, &session)); //1=no run, 0=run
		std::cout << "FPGA open-session status: " << status << std::endl;

		if (NiFpga_IsNotError(status))
		{
			InitializeFPGA(&status, session);

			//run the FPGA application if the FPGA was opened in 'no-run' mode
			//NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));


			//NonDetDigitalPulse(&status, session, 10*ms);
			MainSequence(&status, session);
			//PulseVTcontrol(&status, session, 3 * s, VTback);
			//StartVT(&status, session);

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
	

	//runPIstageSequence();
	//runUARTsequence();

	//WriteTiff();

	getchar();

	return 0;
}