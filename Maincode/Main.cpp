#include "Sequences.h"

int main(int argc, char* argv[])
{
	try
	{
		FPGAns::FPGA fpga;		//Create a FPGA session
		try
		{
			//seq_main(fpga);			
			//seq_contAcquisition(fpga);
			//seq_testPixelclock(fpga);
			//seq_testStageSetPosition();
			//seq_testAODO(fpga);
			//seq_testmPMT();
			//seq_testPockels(fpga);
			//seq_testLaserComm(fpga);
			//seq_testFilterwheel();
			//seq_testRS(fpga);
			//seq_contAcquisitionTest(fpga);
			//seq_testConvertI16toVoltage();
			seq_testGalvoSync(fpga);
			//seq_testTiff();
		}
		catch (const std::invalid_argument &e)
		{
			std::cout << "An invalid argument has occurred in " << e.what() << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getchar();
		}
		catch (const std::overflow_error &e)
		{
			std::cout << "An overflow has occurred in " << e.what() << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getchar();
		}
		catch (const FPGAns::FPGAexception &e)
		{
			std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getchar();
		}
		catch (const std::runtime_error &e)
		{
			std::cout << "A runtime error has occurred in " << e.what() << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getchar();
		}
		catch (...)
		{
			std::cout << "An unknown error has occurred" << std::endl;
			std::cout << "Press any key to continue..." << std::endl;
			getchar();
		}

		fpga.close(0);		//Close the FPGA connection

	}
	//Catch exceptions thrown by the constructor FPGAns::FPGA
	catch (const FPGAns::FPGAexception &e)
	{
		std::cout << "An FPGA exception has occurred in " << e.what() << std::endl;
		std::cout << "Press any key to continue..." << std::endl;
		getchar();
	}
	std::cout << "Press any key to continue..." << std::endl;
	getchar();

	
	return 0;
}