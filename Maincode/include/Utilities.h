#pragma once
#include <sstream>					//For std::ostringstream
#include <iomanip>					//For std::setprecision
#include <fstream>					//For std::ofstream
#include <iostream>
#include <Const.h>
#include <experimental/filesystem>	//standard method in C++14 but not C++11
#include <bitset>					//For  std::bitset
#include <tiffio.h>					//Tiff files				
#include <windows.h>				//For using the ESC key
using namespace Constants;

std::string doesFileExist(const std::string filename);
std::string toString(const double number, const int nDecimalPlaces);
void printHex(int input);
void printHex(const std::vector<uint8_t>  input);
void printHex(const std::string input);
void printBinary16(const int input);
U16 doubleToFx2p14(double n);
void pressAnyKeyToCont();
void pressESCforEarlyTermination();


//For saving the parameters to a text file
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

//For manipulating and saving U8 Tiff images
class TiffU8
{
	U8* mArray;
	int mWidthPerFrame;
	int mHeightPerFrame;
	int mNframes;
	int mBytesPerLine; 
	//int mStripSize;	//I think this was implemented to allow different channels (e.g., RGB) on each pixel
public:
	TiffU8(const std::string filename, const int nframes);
	TiffU8(const U8* inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes);
	TiffU8(const std::vector<U8> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes);
	TiffU8(const int width, const int height, const int nframes);
	~TiffU8();
	U8* const pointerToTiff() const;
	int widthPerFrame() const;
	int heightPerFrame() const;
	int nFrames() const;
	void saveToFile(std::string filename, const MULTIPAGE multipage, const OVERRIDE override = OVERRIDE::DIS, const ZSCAN scanDir = ZSCAN::TOPDOWN) const;
	void mirrorOddFrames();
	void averageEvenOddFrames();
	void averageFrames();
	void analyze() const;
	void saveTxt(const std::string fileName) const;
	void pushImage(const int frameIndex, const U8* inputArray) const;
	void pushImage(const int firstFrameIndex, const int lastFrameIndex, const U8* inputArray) const;
	void mergePMT16Xchannels(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const;
	void correctRSdistortion();
	void Test();
};


class TiffStack
{
	TiffU8 mSameZ;		//For imaging the same z plane many times and then compute the average image
	TiffU8 mDiffZ;		//For imaging different z planes
public:
	TiffStack(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nDiffZ, const int nSameZ);
	void pushSameZ(const int indexSameZ, U8* const pointerToTiff);
	void pushDiffZ(const int indexDiffZ);
	void saveToFile(const std::string filename, OVERRIDE override) const;
};

