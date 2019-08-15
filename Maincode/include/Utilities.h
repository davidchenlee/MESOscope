#pragma once
#include <sstream>					//For std::ostringstream
#include <iomanip>					//For std::setprecision
#include <fstream>					//For std::ofstream
#include <iostream>
#include <Const.h>
#include <experimental/filesystem>	//standard method in C++14 but not C++11
#include <bitset>					//For std::bitset
#include <tiffio.h>					//Tiff files				
#include <windows.h>				//For using the ESC key
#include <CL/cl.hpp>				//OpenCL
using namespace Constants;

std::string doesFileExist(const std::string filename);
std::string toString(const double number, const int nDecimalPlaces);
void printHex(int input);
void printHex(const std::vector<uint8_t>  input);
void printHex(const std::string input);
void printBinary16(const int input);
U16 doubleToFx2p14(double n);
template<class T> inline T clip(T x, T lower, T upper);
template<class T> inline U8 clipU8top(const T x);
template<class T> inline U8 clipU8dual(const T x);
void pressAnyKeyToCont();
void pressESCforEarlyTermination();
double multiply16X(const double input);

//For saving the parameters to a text file
class Logger
{
public:
	Logger(const std::string filename);
	~Logger();
	void record(const std::string description);
	void record(const std::string description, const double input);
	void record(const std::string description, const std::string input);
private:
	std::ofstream mFileHandle;
};

//For manipulating and saving U8 Tiff images
class TiffU8
{
public:
	TiffU8(const std::string filename);
	TiffU8(const U8* inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames);
	TiffU8(const std::vector<U8> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames);
	TiffU8(const int width_pix, const int height_pix, const int nFrames);
	~TiffU8();
	U8* const data() const;
	int widthPerFrame_pix() const;
	int heightPerFrame_pix() const;
	int nFrames() const;
	void splitIntoFrames(const int nFrames);
	void saveToFile(std::string filename, const TIFFSTRUCT tiffStruct, const OVERRIDE override = OVERRIDE::DIS, const SCANZ scanDir = SCANZ::TOPDOWN) const;
	void mirrorOddFrames();
	void mergeFrames();
	void mirror();
	void averageEvenOddFrames();
	void averageFrames();
	void binFrames(const int nFramesPerBin);
	bool isDark(const double threshold) const;
	std::vector<bool> isDark(const double threshold, const int tileWidth_pix, const int tileHeight_pix) const;
	void saveToTxt(const std::string fileName) const;
	void pushImage(const U8* inputArray, const int frameIndex) const;
	void pushImage(const U8* inputArray, const int firstFrameIndex, const int lastFrameIndex) const;
	void mergePMT16Xchan(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const;
	void correctRSdistortionGPU(const double FFOVfast);
	void correctRSdistortionCPU(const double FFOVfast);
	void suppressCrosstalk(const double crosstalkRatio = 1.0);
	void flattenField(const double maxScaleFactor = 1.0);
private:
	U8* mArray;
	int mWidthPerFrame_pix;
	int mHeightPerFrame_pix;
	int mNframes;
	int mBytesPerLine; 
	//int mStripSize;	//I think this was implemented to allow different channels (e.g., RGB) on each pixel
};

class QuickStitcher
{
public:
	QuickStitcher(const int tileWidth, const int tileHeight, const int nRow, const int nCol);
	void push(const TiffU8 &tile, const int rowIndex, const int colIndex);
	void saveToFile(std::string filename, const OVERRIDE override) const;
private:
	TiffU8 mTiff;
	int mNrow;
	int mNcol;
};

/*Obsolete
class TiffStack
{
public:
	TiffStack(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nDiffZ, const int nSameZ);
	void pushSameZ(const int indexSameZ, const U8* data);
	void pushDiffZ(const int indexDiffZ);
	void saveToFile(const std::string filename, OVERRIDE override) const;
private:
	TiffU8 mArraySameZ;		//For imaging the same z plane many times and then compute the average image
	TiffU8 mArrayDiffZ;		//For imaging different z planes
};
*/

