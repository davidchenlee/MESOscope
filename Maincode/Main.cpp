//#include <concrt.h> 	//Concurrency::wait(2000);
#include "Sequences.h"

int main()
{

	FPGAapi fpga;

	//Initialize the FPGA
	fpga.initialize();

	Sequence1(fpga);

	fpga.flushFIFO();
	fpga.close();
	
	std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}