//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"

int main()
{
	FPGAapi fpga;			//Open a FPGA connection
	try
	{
		fpga.initialize();	//Initialize the FPGA
		Sequence1(fpga);	//Run the sequence
		fpga.flushFIFO();	//Flush the FPGA FIFOs as precaution
		fpga.close();		//Close the FPGA connection
	}
	catch (FPGAexception &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (std::invalid_argument &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (std::overflow_error &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (std::runtime_error &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown error has occurred" << std::endl;
	}

	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}