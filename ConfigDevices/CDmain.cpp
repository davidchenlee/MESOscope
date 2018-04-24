#include "Devices.h"

int main()
{

	FPGAapi fpga;

	//Initialize the FPGA
	fpga.initialize();

	ResonantScanner RS(fpga);
	if (1)
		RS.turnOn(200 * um);
	else
		RS.turnOff();

	fpga.flushFIFO();
	fpga.close();
	std::cout << "\nPress any key to continue..." << std::endl;
	getchar();

	return 0;
}