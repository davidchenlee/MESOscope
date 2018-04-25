#include "Devices.h"

int main()
{
	/*
	//To make sure the the filterwheel 1 is set to the correct position
	FilterWheel();
	Sleep(1000); //wait for the filterwheel to settle
	*/

	FPGAapi fpga;

	//Initialize the FPGA
	fpga.initialize();



	ResonantScanner RS(fpga);
	try
	{
		if (1)
			RS.turnOn(200 * um);
		else
			RS.turnOff();

		fpga.flushFIFO();
	}
	catch (...)
	{
		std::cout << "ooops" << std::endl;
	}

	std::cout << "\nPress any key to continue..." << std::endl;
	getchar();

	return 0;
}