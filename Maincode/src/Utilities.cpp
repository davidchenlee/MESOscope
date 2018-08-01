#include "Utilities.h"

//Convert an int to hex and print it out
void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

void printHex(std::vector<uint8_t>  input)
{
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << (int)input[ii];
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << std::endl;
}

//Convert a string to hex and print it out
void printHex(std::string input)
{
	const char *cstr = input.c_str();
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << (int)cstr[ii];
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << std::endl;
}

void printBinary16(int input)
{
	std::cout << std::bitset<16>(input) << std::endl;
}

U16 convertDoubleToFx2p14(double n)
{
	const int FIXED_BIT = 14; //Number of decimal digits. It MUST match the LV implementation: currently fx2.14 (U16 is split into 2 integer digits + 14 decimal digits)
	U16 int_part = 0, frac_part = 0;
	double t;
	int_part = (U16)floor(n) << FIXED_BIT;
	n -= (U16)floor(n);

	t = 0.5;
	for (int i = 0; i < FIXED_BIT; i++) {
		if ((n - t) >= 0) {
			n -= t;
			frac_part += (1 << (FIXED_BIT - 1 - i));
		}
		t = t / 2;
	}

	return int_part + frac_part;
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

void Logger::record(const std::string description)
{
	mFileHandle << "\n";
	mFileHandle << description << std::endl;
}

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << std::endl;
}

void Logger::record(const std::string description, const std::string input)
{
	mFileHandle << description << input << std::endl;
}

