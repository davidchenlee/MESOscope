#include "Routines.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAns::FPGA fpga;		//Create a FPGA session
		try
		{
			//MAIN SEQUENCES
			//MainRoutines::discreteZstageScan(fpga);
			//MainRoutines::discreteZstageScanForLocationList(fpga);
			//MainRoutines::liveScan(fpga);
			//MainRoutines::contZstageScan(fpga);
			//MainRoutines::sequencer(fpga);

			//TESTS
			//TestRoutines::photobleach(fpga);
			//TestRoutines::digitalLatency(fpga);
			//TestRoutines::analogLatency(fpga);
			//TestRoutines::fineTuneGalvoScan(fpga);
			//TestRoutines::pockels(fpga);
			//TestRoutines::galvo(fpga);
			//TestRoutines::pixelclock(fpga);
			//TestRoutines::analogAndDigitalOut(fpga);
			//TestRoutines::analogRamp(fpga);
			//TestRoutines::digitalTiming(fpga);
			//TestRoutines::filterwheel();
			TestRoutines::shutter(fpga);
			//TestRoutines::stagePosition();
			//TestRoutines::stageConfig();
			//TestRoutines::PMT16Xconfig();
			//TestRoutines::lasers(fpga);
			//TestRoutines::virtualLasers(fpga);
			//TestRoutines::pockelsRamp(fpga);
			//TestRoutines::resonantScanner(fpga);
			//TestRoutines::convertI16toVolt();
			//TestRoutines::tiffU8();
			//TestRoutines::ethernetSpeed();
			//TestRoutines::vibratome(fpga);
			//TestRoutines::sequencer();
			//TestRoutines::multithread();
			//TestRoutines::sequencerConcurrentTest();
			//TestRoutines::locationSequencer();
		}
		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred in " << e.what() << "\n";
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred in " << e.what() << "\n";
		}
		catch (const FPGAns::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred in " << e.what() << "\n";
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << "\n";
		}

		fpga.close(NORESET);		//Close the FPGA connection
		pressAnyKeyToCont();

	}
	//Catch exceptions thrown by the constructor FPGAns::FPGA
	catch (const FPGAns::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
		pressAnyKeyToCont();
	}
	//pressAnyKeyToCont();

	return 0;
}