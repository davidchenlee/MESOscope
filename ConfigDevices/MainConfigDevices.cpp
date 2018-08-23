#include "Devices.h"

//argv[1] = FOV, argv[2] = On/Off
int main(int argc, char* argv[])
{
	if (argc < 3) {
		std::cout << "ERROR: not enough arguments" << std::endl;
		return 0;
	}

	try
	{
		FPGAns::FPGA fpga;	//Open a FPGA connection
		try
		{
			ResonantScanner RS(fpga);
			//Shutter shutter1(fpga, SHUTTER1);
			Laser vision;

			//Set the FOV
			int FFOV_um(std::stoi(argv[1]));
			std::string runCommand(argv[2]);

			if (FFOV_um < 0 || FFOV_um > 300)
				throw std::invalid_argument((std::string)__FUNCTION__ + ": RS FFOV must be in the range 0-300 um");

			//Turn the RS On/Off
			if (runCommand == "1")
			{
				RS.turnOn_um(FFOV_um);
				//shutter1.open();
				vision.setShutter(1);
			}
			else if (runCommand == "0")
			{
				RS.turnOff();
				//shutter1.close();
				vision.setShutter(0);
			}
			else
				throw std::invalid_argument((std::string)__FUNCTION__  + ": Invalid command");

		}

		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred: " << e.what() << std::endl;
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred: " << e.what() << std::endl;
		}
		catch (const FPGAns::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred: " << e.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << std::endl;
		}

		fpga.close();		//Close the FPGA connection
	}

	catch (const FPGAns::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred: " << e.what() << std::endl;
	}

	//std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}