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
			QV[0].push(AnalogOut(8*_us, 5*_V));
			QV[0].push(AnalogOut(8*_us, 0*_V));
			//QV[0].push(AnalogOut(1*_ms, 10*_V));
			//QV[0].push(AnalogOut(1*_us, 0*_V);

			U32Q linearRamp1 = linearRamp(8*_us, 1*_ms, 0*_V, 5*_V);
			//QV[0] = ConcatenateQ(QV[0], linearRamp1);
			QV[0].push(AnalogOut(8*_us, 10*_V));
			QV[0].push(AnalogOut(8*_us, 0*_V));//go back to zero
			linearRamp1 = {};

			//AO2
			//QV[1].push(u32pack(us2tick(5*_us), VO0));
			//QV[1].push(u32pack(4*_ms, VO1));
			//QV[1].push(u32pack(4*_us, VO2));
			//QV[1].push(u32pack(4*_us, VO3));

			//DO1
			QV[2].push(DigitalOut(8*_us, 0x0001));
			QV[2].push(DigitalOut(8*_us, 0x0000));
			//QV[2].push(u32pack(1*_ms, 0x0000));
			QV[2].push(DigitalOut(8*_us, 0x0001));
			QV[2].push(DigitalOut(8*_us, 0x0000));

			/*
			//linear output
			U32Q linearRamp1= linearRamp(4 * _ms, 1 * _s, 0*_V, -5 * _V);
			U32Q linearRamp2 = linearRamp(4 * _ms, 1 * _s, -5*_V, 5 * _V);
			U32Q linearRamp3 = linearRamp(4 * _ms, 1 * _s, 5 * _V, 0 * _V);
			QV[0] = ConcatenateQ(ConcatenateQ(linearRamp1, linearRamp2), linearRamp3);//overwrites FIFO[0] with a linear ramp
			linearRamp1, linearRamp2, linearRamp3 = {};
			*/

			SendOutQueue(&status, session, QV);
			PulseTrigger(&status, session);


			//SECOND ROUND
			if (0)
			{
				U32QV QV2 (Nchannels);

				double At0 = 4 * _us;//40 tick = 1 us
				double At1 = 4 * _us;
				int16_t VO0 = AOUT(10 * _V);
				int16_t VO1 = AOUT(0);

				//AO1
				QV2[0].push(u32pack(At0, VO0));
				QV2[0].push(u32pack(At1, VO1));
				//AO2
				QV2[1].push(u32pack(At0, VO0));
				QV2[1].push(u32pack(At1, VO1));
				//DO1
				QV2[2].push(u32pack(4*_us, 0x0001));
				QV2[2].push(u32pack(4*_us, 0x0000));

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
