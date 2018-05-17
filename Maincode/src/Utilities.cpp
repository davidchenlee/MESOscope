#include "Utilities.h"

void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

std::string toString(const double number, const int nDecimalPlaces)
{
	std::ostringstream str;
	str << std::fixed << std::setprecision(nDecimalPlaces);
	str << number;
	return str.str();
}

//Check if the file already exists
std::string file_exists(const std::string filename)
{
	std::string suffix = "";

	for (int ii = 1; std::experimental::filesystem::exists(foldername + filename + suffix + ".tif"); ii++)
		suffix = " (" + std::to_string(ii) + ")";

	return filename + suffix;
}


Logger::Logger(const std::string filename)
{
	mFileHandle.open(foldername + filename + ".txt");
};

Logger::~Logger()
{
	mFileHandle.close();
};

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << std::endl;
}