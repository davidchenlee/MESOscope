//#include <math.h>
//#include <time.h>
//#include <concrt.h> 	//Concurrency::wait(2000);
#include "windows.h"	//the stages use this lib. also Sleep
#include "FPGA.h"
//using namespace std;


int main()
{
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

			//Create a vector of queues. Assigns a queue to each channel
			U32QV QV (Nchannels);

			//AO1
			QV[0].push(AnalogOut(4*_us, 10));
			QV[0].push(AnalogOut(4*_us, 0));
			//QV[0].push(AnalogOut(1*_ms, 10));
			//QV[0].push(AnalogOut(1*_us, 0);

			QV[0] = ConcatenateQ(QV[0], linearRamp(4*_us, 1*_ms, 0, 5));
			//QV[0].push(AnalogOut(1*_ms, 0));
			QV[0].push(AnalogOut(4*_us, 5));
			QV[0].push(AnalogOut(4*_us, 0));//go back to zero

			//AO2
			//QV[1].push(u32pack(us2tick(5*_us), VO0));
			//QV[1].push(u32pack(4*_ms, VO1));
			//QV[1].push(u32pack(4*_us, VO2));
			//QV[1].push(u32pack(4*_us, VO3));

			//DO1
			QV[2].push(DigitalOut(4*_us, 1));
			QV[2].push(DigitalOut(4*_us, 0));
			QV[2].push(DigitalOut(1*_ms, 0));
			QV[2].push(DigitalOut(4*_us, 1));
			QV[2].push(DigitalOut(4*_us, 0));

			/*
			//linear output
			U32Q linearRamp1= linearRamp(4*_ms, 1*_s, 0, -5);
			U32Q linearRamp2 = linearRamp(4*_ms, 1*_s, -5, 5);
			U32Q linearRamp3 = linearRamp(4*_ms, 1*_s, 5, 0);
			QV[0] = ConcatenateQ(ConcatenateQ(linearRamp1, linearRamp2), linearRamp3);//overwrites FIFO[0] with a linear ramp
			linearRamp1, linearRamp2, linearRamp3 = {};
			*/

			SendOutQueue(&status, session, QV);
			PulseTrigger(&status, session);


			//SECOND ROUND
			if (0)
			{
				U32QV QV2 (Nchannels);

				//AO1
				QV2[0].push(AnalogOut(4*_us, 10));
				QV2[0].push(AnalogOut(4*_us, 0));
				//AO2
				QV2[1].push(AnalogOut(4*_us, 10));
				QV2[1].push(AnalogOut(4*_us, 0));
				//DO1
				QV2[2].push(DigitalOut(4*_us, 1));
				QV2[2].push(DigitalOut(4*_us, 0));

				SendOutQueue(&status, session, QV2);
				PulseTrigger(&status, session);
			}

			//EVIL FUNCTION. DO NOT USE
			/* Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target)
			unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.*/
			//NiFpga_MergeStatus(&status, NiFpga_Close(session, 1)); //0 resets, 1 does not reset

		}


		/* You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.*/
		NiFpga_MergeStatus(&status, NiFpga_Finalize());
		std::cout << "FPGA finalize status: " << status << "\n";
		Sleep(2000);
	}

	return 0;
}
