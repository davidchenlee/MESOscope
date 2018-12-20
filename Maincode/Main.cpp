#include "Routines.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAns::FPGA fpga;		//Create a FPGA session
		try
		{
			//mainVision(fpga);
			//mainFidelity(fpga);
			testContinuousXYZacq(fpga);
			//testGalvo(fpga);
			//testPixelclock(fpga);
			//testAODO(fpga);
			//testAOramp(fpga);
			//checkDigitalTiming(fpga);
			//calibDigitalLatency(fpga);
			//calibAnalogLatency(fpga);
			//testFilterwheel();
			//testShutter(fpga);
			//testStagePosition();
			//testStageConfig();
			//testPMT16X();
			//testLaser(fpga);
			//testVirtualLaser(fpga);
			//testPockels(fpga);
			//testRS(fpga);
			//testConvertI16toVolt();
			//testTiffU8();
			//testEthernetSpeed();
			//testVibratome(fpga);
			//testSequencer();
		}
		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred in " << e.what() << "\n";
			std::cout << "Press any key to continue...\n";
			getchar();
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred in " << e.what() << "\n";
			std::cout << "Press any key to continue...\n";
			getchar();
		}
		catch (const FPGAns::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
			std::cout << "Press any key to continue...\n";
			getchar();
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred in " << e.what() << "\n";
			std::cout << "Press any key to continue...\n";
			getchar();
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << "\n";
			std::cout << "Press any key to continue...\n";
			getchar();
		}

		fpga.close(NORESET);		//Close the FPGA connection

	}
	//Catch exceptions thrown by the constructor FPGAns::FPGA
	catch (const FPGAns::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
		std::cout << "Press any key to continue...\n";
		getchar();
	}
	//std::cout << "Press any key to continue...\n";
	//getchar();

	return 0;
}