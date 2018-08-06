#pragma once
#include <sstream>					//For std::ostringstream
#include <iomanip>					//For std::setprecision
#include <fstream>					//For std::ofstream
#include <iostream>
#include <Const.h>
#include <experimental/filesystem>	//standard method in C++14 but not C++11
#include <bitset>					//For  std::bitset
using namespace Parameters;

std::string file_exists(const std::string filename);
std::string toString(const double number, const int nDecimalPlaces);
void printHex(int input);
void printHex(std::vector<uint8_t>  input);
void printHex(std::string input);
void printBinary16(int input);
U16 convertDoubleToFx2p14(double n);

class Logger
{
	std::ofstream mFileHandle;
public:
	Logger(const std::string filename);
	~Logger();
	void record(const std::string description);
	void record(const std::string description, const double input);
	void record(const std::string description, const std::string input);
};