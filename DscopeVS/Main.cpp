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
			//SendOutQueue(&status, session, GalvoTest());
			PulseTrigger(&status, session);

			
			// start acquiring data
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 1));
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0));
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Read_data, 1));
			NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Read_data, 0));
			CountPhotons(&status, session);

			
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
