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




	fpga.close();
	std::cout << "\nPress any key to continue..." << std::endl;
	getchar();

	return 0;
}