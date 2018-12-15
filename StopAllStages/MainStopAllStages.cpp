#include "Devices.h"

int main(int argc, char* argv[])
{
	try
	{
		Stage stage;
		stage.stopALL();
	}

	catch (const std::invalid_argument &e)
	{
		std::cout << "An invalid argument has occurred: " << e.what() << "\n";
	}
	catch (const std::overflow_error &e)
	{
		std::cout << "An overflow has occurred: " << e.what() << "\n";
	}
	catch (const std::runtime_error &e)
	{
		std::cout << "A runtime error has occurred: " << e.what() << "\n";
	}
	catch (...)
	{
		std::cout << "An unknown error has occurred\n";
	}

	std::cout << "\nPress any key to continue...\n";
	getchar();

	return 0;
}