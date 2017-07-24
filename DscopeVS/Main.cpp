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
			U32QV Qarray (Nchannels);

			//AO1
			tt_t At0 = us2tick(4 * _us);//40 tick = 1 us
			tt_t At1 = us2tick(1400 * _us);
			tt_t At2 = us2tick(4 * _us);
			tt_t At3 = us2tick(4 * _us);
			int16_t VO0 = AOUT(10 * _V);
			int16_t VO1 = AOUT(0 * _V);
			int16_t VO2 = AOUT(0 * _V);
			int16_t VO3 = AOUT(0 * _V);
			Qarray[0].push(u32pack(At0, VO0));
			Qarray[0].push(u32pack(At1, VO1));
			Qarray[0].push(u32pack(At2, VO2));
			Qarray[0].push(u32pack(At3, VO3));

			//AO2
			Qarray[1].push(u32pack(us2tick(5 * _us), VO0));
			Qarray[1].push(u32pack(At1, VO1));
			Qarray[1].push(u32pack(At2, VO2));
			Qarray[1].push(u32pack(At3, VO3));

			//DO1
			tt_t Dt0 = us2tick(4 * _us); //40 tick = 1 us
			tt_t Dt1 = us2tick(1 * _ms);
			tt_t Dt2 = us2tick(4 * _us);
			tt_t Dt3 = us2tick(4 * _us);
			Qarray[2].push(u32pack(Dt0, 0x0001));
			Qarray[2].push(u32pack(Dt1, 0x0000));
			Qarray[2].push(u32pack(Dt2, 0x0001));
			Qarray[2].push(u32pack(Dt3, 0x0000));


			//linear output
			U32Q linearRampUp = linearRamp(4 * _us, 0 * _us, 1 * _ms, 0*_V, 10 * _V);
			U32Q linearRampDown = linearRamp(4 * _us, 0 * _us, 1 * _ms, 10*_V, 0 * _V);
			Qarray[0] = ConcatenateQ(linearRampUp, linearRampDown);//overwrites FIFO[0] with a linear ramp


			SendOutQueue(&status, session, Qarray);
			PulseTrigger(&status, session);


			//SECOND ROUND
			if (1)
			{
				U32QV Qarray2 (Nchannels);

				tt_t At0 = us2tick(4 * _us);//40 tick = 1 us
				tt_t At1 = us2tick(4 * _us);
				int16_t VO0 = AOUT(5 * _V);
				int16_t VO1 = AOUT(0);

				//AO1
				Qarray2[0].push(u32pack(At0, VO0));
				Qarray2[0].push(u32pack(At1, VO1));
				//AO2
				Qarray2[1].push(u32pack(At0, VO0));
				Qarray2[1].push(u32pack(At1, VO1));
				//DO1
				Qarray2[2].push(u32pack(Dt0, 0x0001));
				Qarray2[2].push(u32pack(Dt1, 0x0000));

				SendOutQueue(&status, session, Qarray2);
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
