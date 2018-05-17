#include "Utilities.h"


std::string toString(const double number, const int nDecimalPlaces)
{
	std::ostringstream str;
	str << std::fixed << std::setprecision(nDecimalPlaces);
	str << number;
	return str.str();
}

void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
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