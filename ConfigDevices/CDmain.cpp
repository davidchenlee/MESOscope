#include "Devices.h"


void turnRSon(NiFpga_Status* status, NiFpga_Session session)
{
	const double RSamplitude_um = 200 * um;
	const double RSamplitude_volt = RSamplitude_um * RS_voltPerUm;

	resonantScanner_StartStop(status, session, 0);	//make sure that disable is on
	Sleep(10);
	resonantScanner_SetOutputVoltager(status, session, RSamplitude_volt);
	Sleep(10);
	resonantScanner_StartStop(status, session, 1);
}

void turnRSoff(NiFpga_Status* status, NiFpga_Session session)
{
	resonantScanner_StartStop(status, session, 0);
	Sleep(10);
	resonantScanner_SetOutputVoltager(status, session, 0);
}


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
			if(0)
				turnRSon(&status, session);
			else
				turnRSoff(&status, session);
			
			
			NiFpga_MergeStatus(&status, NiFpga_Close(session, 1));			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
																			//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
																			//0 resets, 1 does not reset
		}

		//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << std::endl;

	}



	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}