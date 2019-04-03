#include "Routines.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAns::FPGA fpga;		//Create a FPGA session
		try
		{
			//MAIN SEQUENCES
			//MainRoutines::frameByFrameScan(fpga);
			//MainRoutines::frameByFrameScan_LocationList(fpga, 2);//scan frame by frame the specified set of locations
			//MainRoutines::liveScan(fpga);//Image nonstop and move the stage manually thru the PI software
			//MainRoutines::continuousScan(fpga);//Scan the z stage continuously. FIX: laser power scaling. Only linear scaling in voltage has been implemented, but not in laser power
			//MainRoutines::sequencer(fpga);FIX: laser power scaling. Only linear scaling in voltage has been implemented, but not in laser power. Also I changed Pinc to be the power increase per um instead of per stack. I have to update this routine to reflect this change.

			//TESTS.
			TestRoutines::demultiplexing(fpga);
			//TestRoutines::digitalLatency(fpga);
			//TestRoutines::analogLatency(fpga);
			//TestRoutines::pockels(fpga);
			//TestRoutines::pockelsRamp(fpga);
			//TestRoutines::photobleach(fpga);
			//TestRoutines::galvosSyncPartialFrame(fpga);
			//TestRoutines::galvosSyncFullFrame(fpga);
			//TestRoutines::fineTuneGalvoScan(fpga);
			//TestRoutines::pixelclock(fpga);
			//TestRoutines::analogAndDigitalOut(fpga);
			//TestRoutines::analogRamp(fpga);
			//TestRoutines::digitalTiming(fpga);
			//TestRoutines::filterwheel();
			//TestRoutines::shutter(fpga);
			//TestRoutines::stagePosition();
			//TestRoutines::stageConfig();
			//TestRoutines::PMT16Xconfig();
			//TestRoutines::lasers(fpga);
			//TestRoutines::virtualLasers(fpga);
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
		//pressAnyKeyToCont();

	}
	//Catch exceptions thrown by the constructor FPGAns::FPGA
	catch (const FPGAns::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << "\n";
		pressAnyKeyToCont();
	}

	pressAnyKeyToCont();
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
