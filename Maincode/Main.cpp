#include "Sequences.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAapi::Session fpga;		//Open the FPGA connection

		try
		{
			fpga.initialize();		//Initialize the FPGA

			//seq_main(fpga);			//Run the sequence
			seq_testPixelclock(fpga);
			//seq_testStageSetPosition();
			//seq_testAODO(fpga);
			//seq_testmPMT();
			//seq_testPockels(fpga);
			//seq_testLaserComm(fpga);
			//seq_testFilterwheel();
			//seq_testRS(fpga);
		}
		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred in " << e.what() << std::endl;
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred in " << e.what() << std::endl;
		}
		catch (const FPGAapi::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred in " << e.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << std::endl;
		}

		fpga.close(0);		//Close the FPGA connection

	}

	//Catch exceptions thrown by the constructor FPGAapi::Session
	catch (const FPGAapi::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
	}
	
	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}