#pragma once
#include <sstream>	//For std::ostringstream
#include <iomanip>	//For std::setprecision
#include <fstream>	//For std::ofstream
#include <iostream>
#include <Const.h>
using namespace Parameters;

std::string toString(const double number, const int nDecimalPlaces);

void printHex(int input);

class Logger
{
	std::ofstream mFileHandle;
public:
	Logger(const std::string filename);
	~Logger();
	void record(const std::string description, const double input);
};
