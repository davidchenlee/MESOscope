#include "Devices.h"

//argv[0] = Program's name, argv[1] = which laser, argv[2] = FOV, argv[3] = On/Off
int main(int argc, char* argv[])
{
	if (argc < 4) {
		std::cout << "ERROR: not enough arguments\n";
		return 0;
	}

	try
	{
		FPGA fpga;	//Open a FPGA connection
		try
		{
			RTseq rtseq{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };
			ResonantScanner RS{ rtseq };
			Laser vision{ Laser::ID::VISION };
			Laser fidelity{ Laser::ID::FIDELITY };

			std::string whichLaser{ argv[1] };			//V for Vision, F for Fidelity VF for both
			double FFOV{ 1.*std::stoi(argv[2]) / um };	//Field of view in um
			std::string runCommand{ argv[3] };			//1 for run RS, 0 for stop RS

			if (FFOV < 0 || FFOV > 300)
				throw std::invalid_argument((std::string)__FUNCTION__ + ": RS FFOV must be in the range 0-300 um");

			//Turn the RS On/Off
			if (runCommand == "1")
			{
				if (whichLaser == "V" || whichLaser == "v")
				{
					vision.setShutter(true);
				}
				else if (whichLaser == "F" || whichLaser == "f")
				{
					fidelity.setShutter(true);
				}
				else if (whichLaser == "B" || whichLaser == "b")
				{
					vision.setShutter(true);
					fidelity.setShutter(true);
				}
				else
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected laser is not available");

				RS.turnOn(FFOV);
			}
			else if (runCommand == "0")
			{
				RS.turnOff();
				vision.setShutter(0);
				fidelity.setShutter(0);
			}
			else
				throw std::invalid_argument((std::string)__FUNCTION__  + ": Invalid command");

		}

		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred: " << e.what() << "\n";
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred: " << e.what() << "\n";
		}
		catch (const FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred: " << e.what() << "\n";
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred\n";
		}

		fpga.close();		//Close the FPGA connection
	}

	catch (const FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred: " << e.what() << "\n";
	}

	//std::cout << "\nPress any key to continue...\n";
	//getchar();

	return 0;
}