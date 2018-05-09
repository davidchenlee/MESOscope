#include "Devices.h"

int main(int argc, char* argv[])
{
	if (argc < 3) {
		std::cout << "ERROR: not enough arguments" << std::endl;
		return 0;
	}

	FPGAapi fpga;			//Open a FPGA connection
	try
	{
		fpga.initialize();	//Initialize the FPGA

		ResonantScanner RS(fpga);
		Shutter shutter1(fpga, Shutter1);

		int FFOV_um(std::stoi(argv[1]));
		std::string runCommand(argv[2]);

		if (FFOV_um < 0 || FFOV_um > 300) throw std::invalid_argument("invalid FFOV");

		if (runCommand == "1")
		{
			RS.turnOn_um(FFOV_um * um);
			std::cout << "RS FFOV set to: " << FFOV_um << " um" << std::endl;
			//shutter1.open();
		}
		else if (runCommand == "0")
		{
			RS.turnOff();
			//shutter1.close();
		}
		else
		{
			throw std::invalid_argument("invalid start/stop command");
		}
			
		fpga.close(0);		//Close the FPGA connection

	}
	catch (const FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred: " << e.what() << std::endl;
	}

	catch (const std::invalid_argument &e)
	{
		std::cout << "An invalid argument has occurred: " << e.what() << std::endl;
	}
	catch (const std::overflow_error &e)
	{
		std::cout << "An overflow has occurred: " << e.what() << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cout << "A runtime error has occurred: " << e.what() << std::endl;
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

	//std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}