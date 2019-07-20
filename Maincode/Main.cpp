#include "Routines.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAns::FPGA fpga;		//Create a FPGA session
		try
		{
			//PMT1X SEQUENCES
			//PMT1XRoutines::sequencer(fpga);//FIX: I changed Pinc to be the power increase per um instead of per stack. I have to update this routine to reflect this change.

			//PMT16X SEQUENCES
			//PMT16XRoutines::PMT16XframeByFrameScan(fpga);
			//PMT16XRoutines::PMT16XframeByFrameScanTest(fpga);
			//PMT16XRoutines::frameByFrameScanTiling(fpga, 1);
			//PMT16XRoutines::liveScan(fpga);
			//PMT16XRoutines::continuousScan(fpga);

			//TESTS
			//TestRoutines::digitalLatency(fpga);
			//TestRoutines::analogLatency(fpga);
			//TestRoutines::pixelclock(fpga);
			//TestRoutines::digitalTiming(fpga);
			//TestRoutines::analogAndDigitalOut(fpga);
			//TestRoutines::analogRamp(fpga);

			//TestRoutines::fineTuneScanGalvo(fpga);
			//TestRoutines::resonantScanner(fpga);
			//TestRoutines::galvosSync(fpga);

			//TestRoutines::stagePosition();
			//TestRoutines::stageConfig();

			//TestRoutines::shutter(fpga);
			//TestRoutines::pockels(fpga);
			//TestRoutines::pockelsRamp(fpga);
			//TestRoutines::lasers(fpga);
			//TestRoutines::virtualLasers(fpga);

			//TestRoutines::convertI16toVolt();
			TestRoutines::tiffU8();
			//TestRoutines::ethernetSpeed();
			//TestRoutines::multithread();

			//TestRoutines::sequencerConcurrentTest();
			//TestRoutines::locationSequencer();

			//TestRoutines::PMT16Xconfig();
			//TestRoutines::PMT16Xdemultiplex(fpga);
			//TestRoutines::PMT16XgavosSyncAndLaser(fpga);

			//TestRoutines::vibratome(fpga);
			//TestRoutines::filterwheel();
			//TestRoutines::collectorLens();
			//TestRoutines::photobleach(fpga);
			//TestRoutines::generateLocationsForBigStitcher();
		}
		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred in " << e.what() << "\n";
			pressAnyKeyToCont();
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred in " << e.what() << "\n";
			pressAnyKeyToCont();
		}
		catch (const FPGAns::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
			pressAnyKeyToCont();
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred in " << e.what() << "\n";
			pressAnyKeyToCont();
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << "\n";
			pressAnyKeyToCont();
		}

		fpga.close(FPGARESET::DIS);		//Close the FPGA connection
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

/*
//Main without calling the fpga, because LV blocks the access to the fpga
int main(int argc, char* argv[])
{
	try
	{
		//Make sure first that the power supply of the FPGA is up
		TestRoutines::PMT16Xconfig();
	}
	catch (...)
	{
		std::cout << "An error has occurred" << "\n";
	}
	pressAnyKeyToCont();
}
*/
