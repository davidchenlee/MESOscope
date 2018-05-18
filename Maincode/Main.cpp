//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"

int main(int argc, char* argv[])
{
	FPGAapi::FPGAsession fpga;			//Open a FPGA connection
	try
	{
		fpga.initialize();	//Initialize the FPGA
		
		seq_main(fpga);		//Run the sequence
		//seq_burnSample(fpga);
		//seq_testStages(fpga);

		fpga.flushBRAMs();  //As precaution, flush the RAM buffers on the FPGA
		fpga.close(0);		//Close the FPGA connection

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
			//Close and reset the FPGA connection. Otherwise, residual data will remain in the FPGA and will probably crash the next sequence and the entire computer as well
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
	getchar();

	return 0;
}