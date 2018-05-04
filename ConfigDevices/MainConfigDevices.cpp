#include "Devices.h"

#define RUN 0


int main(int argc, char* argv[])
{
	FPGAapi fpga;			//Open a FPGA connection
	try
	{
		fpga.initialize();	//Initialize the FPGA

		ResonantScanner RS(fpga);
		Shutter shutter1(fpga, Shutter1);


		if (RUN)
		{
			RS.turnOn_um(100 * um);
			//shutter1.open();
		}
		else
		{
			RS.turnOff();
			//shutter1.close();
		}
			

		fpga.close(0);		//Close the FPGA connection

	}
	catch (const FPGAexception &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}

	catch (const std::invalid_argument &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (const std::overflow_error &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cout << "An error has occurred in " << e.what() << std::endl;
		try
		{
			//Close and reset the FPGA connection. Otherwise, residual data will remain in the FPGA and will probably crash the next sequence and the entire computer as well
			const bool enforceReset = 1;	//DO NOT comment this line out!!
			fpga.close(enforceReset);
		}
		catch (const FPGAexception &e)
		{
			std::cout << "An error has occurred in " << e.what() << std::endl;
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