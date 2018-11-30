#pragma once
#include <sstream>					//For std::ostringstream
#include <iomanip>					//For std::setprecision
#include <fstream>					//For std::ofstream
#include <iostream>
#include <Const.h>
#include <experimental/filesystem>	//standard method in C++14 but not C++11
#include <bitset>					//For  std::bitset
#include <tiffio.h>					//Tiff files
using namespace Constants;

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


class TiffU8
{
	unsigned char* mArray;
	int mWidthPerFrame;
	int mHeightPerFrame;
	int mNframes;
	int mBytesPerLine; 
	//int mStripSize;	//I think this is originally implemented to allow different channels (e.g., RGB) at each pixel
public:
	TiffU8(const std::string filename, const int nframes);
	TiffU8(const unsigned char* inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes);
	TiffU8(const std::vector<unsigned char> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes);
	TiffU8(const int width, const int height, const int nframes);
	~TiffU8();
	unsigned char* const accessTiff() const;
	void saveToFile(std::string filename, const TiffPageStructSelector pageStructFlag, const OverrideFileSelector overrideFlag = NOOVERRIDE) const;
	void mirrorOddFrames();
	void averageEvenOdd();
	void average();
	void analyze() const;
	void saveTxt(const std::string fileName) const;
	void pushImage(const int frame, const unsigned char* inputArray) const;
};