#include "Devices.h"

int run()
{
	//serial port
	string port;
	unsigned long baud;
	size_t bytesWrote;
	int timeout;

	if (0)
	{
		port = "COM6";
		baud = 115200;
		timeout = 150;
	}
	else
	{
		port = "COM1";
		baud = 19200;
		timeout = 100;

	}
	
	// port, baudrate, timeout in milliseconds
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(timeout));

	string TxBuffer = "pos?\r";
	//bytesWrote = my_serial.write(TxBuffer);
	bytesWrote = my_serial.write("?VW\r");

	cout << "Bytes written: " << bytesWrote << endl;

	cout << "Is the serial port open?";
	if (my_serial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;

	string RxBuffer;
	size_t bytesRead = my_serial.read(RxBuffer, 256);


	//Delete echoed message
	std::string::size_type i = RxBuffer.find(TxBuffer);
	if (i != std::string::npos)
		RxBuffer.erase(i, TxBuffer.length());


	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\r'), RxBuffer.end());
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '\n'), RxBuffer.end());
	RxBuffer.erase(std::remove(RxBuffer.begin(), RxBuffer.end(), '>'), RxBuffer.end());
	//std::replace(RxBuffer.begin(), RxBuffer.end(), '\r', '\n');

	cout << RxBuffer;

	//getchar();

	return 0;
}

int main(int argc, char* argv[])
{
	if (argc < 2) {
		std::cout << "ERROR: not enough arguments" << std::endl;
		return 0;
	}

	FPGAapi fpga;			//Open a FPGA connection
	try
	{
		fpga.initialize();	//Initialize the FPGA

		ResonantScanner RS(fpga);
		Shutter shutter1(fpga, Shutter1);

		std::string input(argv[1]);
		if (input == "1")
		{
			RS.turnOn_um(200 * um);
			//shutter1.open();
		}
		else
		{
			RS.turnOff();
			//shutter1.close();
			run();
			//testFilterWheel();
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

	//std::cout << "\nPress any key to continue..." << std::endl;
	//getchar();

	return 0;
}