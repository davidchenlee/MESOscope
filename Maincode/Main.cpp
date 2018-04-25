//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"
#include <exception>


int main()
{

	FPGAapi fpga;
	fpga.initialize();	//Initialize the FPGA

	try
	{
		ResonantScanner RS(fpga);
		if (1)
			RS.turnOn(200 * um);
		else
			RS.turnOff();

		Sequence1(fpga);
		fpga.flushFIFO();



	}
	catch (MyException &e)
	{
		std::cout << "An error has been caught" << std::endl;
		std::cout << e.what() << std::endl;
	}
	catch (std::runtime_error &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}


	fpga.close();
	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}