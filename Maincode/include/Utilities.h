#pragma once
#include <filesystem>				//For creating directories
#include <sstream>					//For std::ostringstream
#include <iomanip>					//For std::setprecision
#include <fstream>					//For std::ofstream
#include <iostream>
#include <Const.h>
#include <bitset>					//For std::bitset
#include <tiffio.h>					//Tiff files				
#include <windows.h>				//For using the ESC key
#include <CL/cl.hpp>				//OpenCL
#include <conio.h>					//For _getch()
using namespace Constants;

namespace Util
{
	std::string doesFileExist(const std::string folderPath, const std::string filename, const std::string fileExtension);
	std::string toString(const double number, const int nDecimalPlaces);
	void printHex(int input);
	void printHex(const std::vector<uint8_t>  input);
	void printHex(const std::string input);
	void printBinary16(const int input);
	U16 doubleToFx2p14(double n);
	int intceil(const double input);
	int convertScandirToInt(const SCANDIR scanDir);
	double exponentialFunction(const double Pmin, const double depthZ, const double decayLengthZ);
	template<class T> inline T clip(T x, T lower, T upper);
	template<class T> inline U8 clipU8top(const T x);
	template<class T> inline U8 clipU8dual(const T x);
	void pressAnyKeyToCont();
	void pressESCforEarlyTermination();
	void pressAnyKeyToContOrESCtoExit();
	double multiply16X(const double input);
	std::string convertWavelengthToFluorMarker_s(const int wavelength_nm);
	bool isBright(const std::vector<bool> vec_boolmap, const TILEDIM2 tileArraySizeIJ, const TILEIJ tileIndicesIJ);
	void saveBoolmapToText(const std::string folderPath, std::string filename, const std::vector<bool> vec_boolmap, const TILEDIM2 tileArraySizeIJ, const OVERRIDE override);
	int determineNumberOf1s(const std::vector<bool> vec_boolmap);
}

//For saving the parameters to a text file
class Logger final
{
public:
	Logger(const std::string folderPath, std::string filename, const OVERRIDE override);
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
	TiffU8(const std::string folderPath, const std::string filename);
	TiffU8(const TiffU8& tiff);
	TiffU8(const U8* inputArray, const int heightPerFrame_pix, const int widthPerFrame_pix, int nFrames = 1);
	TiffU8(const std::vector<U8> &inputImage, const int heightPerFrame_pix, const int widthPerFrame_pix, const int nFrames = 1);
	TiffU8(const int height_pix, const int width_pix, const int nFrames = 1);
	~TiffU8();
	U8* const data() const;
	int readHeightPerFrame_pix() const;
	int readWidthPerFrame_pix() const;
	int readNpixPerFrame_pix() const;
	int readNframes() const;

	void pushImage(const U8* inputArray, const int frameIndex) const;
	void pushImage(const U8* inputArray, const int indexFirstFrame, const int indexLastFrame) const;
	void splitFrames(const int nFrames);
	void mergeFrames();
	void mergePMT16Xchan(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const;
	void saveToFile(const std::string folderPath, std::string filename, const TIFFSTRUCT tiffStruct, const OVERRIDE override, const SCANDIR scanDirZ = SCANDIR::UPWARD) const;
	void saveToTxt(const std::string folderPath, const std::string fileName) const;

	void mirrorOddFrames();
	void mirrorSingleFrame();
	void averageEvenOddFrames();
	void averageFrames();
	void binFrames(const int nFramesPerBin);
	void correctRSdistortionGPU(const double FFOVfast);
	void correctRSdistortionCPU(const double FFOVfast);
	void correctFOVslowCPU(const double FFOVfast);
	void suppressCrosstalk(const double crosstalkRatio = 1.0);
	void flattenFieldLinear(const double scaleFactor, const int lowerChan, const int higherChan);
	void flattenFieldGaussian(const double expFactor);

	void loadTiffU8(const std::string folderPath, const std::string filename);

private:
	U8* mArray;
	int mHeightPerFrame_pix;
	int mWidthPerFrame_pix;
	int mNframes;
	int mBytesPerLine; 
	int mNpixPerFrame;	//Total number of pixels in a frame
	int mNpixAllFrames;	//Total number of pixels in all the frames
	//int mStripSize;	//I think this was implemented to allow different channels (e.g., RGB) on each pixel
};