#include "Sequences.h"

int main(int argc, char* argv[])
{
	FPGAapi::Session fpga;		//Open a FPGA connection
	try
	{
		fpga.initialize();		//Initialize the FPGA
		
		seq_main(fpga);			//Run the sequence
		//seq_testStages(fpga);

		fpga.close(0);			//Close the FPGA connection under normal conditions

	}
	catch (const FPGAapi::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
	}

	catch (const std::invalid_argument &e)
	{
		std::cout << "An invalid argument has occurred in " << e.what() << std::endl;
	}
	catch (const std::overflow_error &e)
	{
		std::cout << "An overflow has occurred in " << e.what() << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cout << "A runtime error has occurred in " << e.what() << std::endl;
		try
		{
			//Reset the FPGA. Otherwise residual data will remain in the FPGA and will probably crash the computer in the next run
			const bool enforceReset = 1;
			fpga.close(enforceReset); //DO NOT comment out this line!!
		}
		catch (const FPGAapi::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
		}

	}
	catch (...)
	{
		std::cout << "An unknown error has occurred" << std::endl;
	}

	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}