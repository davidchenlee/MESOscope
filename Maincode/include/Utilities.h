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
#include <conio.h>					//For _getch()
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
void pressAnyKeyToContOrESCtoExit();
double multiply16X(const double input);
int SCANDIRtoInt(const SCANDIR scanDir);
void reverseSCANDIR(SCANDIR &scanDir);
double determineInitialScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineFinalScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineInitialLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);
double determineFinalLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);

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
	TiffU8(const U8* inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames = 1);
	TiffU8(const std::vector<U8> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames = 1);
	TiffU8(const int width_pix, const int height_pix, const int nFrames = 1);
	~TiffU8();
	U8* const data() const;
	int widthPerFrame_pix() const;
	int heightPerFrame_pix() const;
	int nFrames() const;

	void pushImage(const U8* inputArray, const int frameIndex) const;
	void pushImage(const U8* inputArray, const int indexFirstFrame, const int indexLastFrame) const;
	void splitFrames(const int nFrames);
	void mergeFrames();
	void mergePMT16Xchan(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const;
	void saveToFile(std::string filename, const TIFFSTRUCT tiffStruct, const OVERRIDE override = OVERRIDE::DIS, const SCANDIR scanDirZ = SCANDIR::UPWARD) const;
	void saveToTxt(const std::string fileName) const;

	void mirrorOddFrames();
	void mirror();
	void averageEvenOddFrames();
	void averageFrames();
	void binFrames(const int nFramesPerBin);
	void correctRSdistortionGPU(const double FFOVfast);
	void correctRSdistortionCPU(const double FFOVfast);
	void correctFOVslowCPU(const double FFOVfast);
	void suppressCrosstalk(const double crosstalkRatio = 1.0);
	void flattenField(const double scaleFactor, const int lowerChan, const int higherChan);
private:
	U8* mArray;
	int mWidthPerFrame_pix;
	int mHeightPerFrame_pix;
	int mNframes;
	int mBytesPerLine; 
	//int mStripSize;	//I think this was implemented to allow different channels (e.g., RGB) on each pixel
};

//II is the row index and JJ is the column index of the tile wrt the stitched image
class BoolMap
{
public:
	BoolMap(const TiffU8 &tiff, const int tileWidth_pix, const int tileHeight_pix, const double threshold);
	bool isTileBright(const int II, const int JJ);
	void saveTileMapToText(std::string filename);
	void SaveTileGridOverlap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
	void saveTileMap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
private:
	const TiffU8 &mTiff;
	double mThreshold;				//Threshold for generating the boolmap
	int mWidth_pix;					//Pixel width of the stitched image
	int mHeight_pix;				//Pixel height of the stitched image
	int mTileWidth_pix;				//Pixel width of a tile
	int mTileHeight_pix;			//Pixel height of a tile
	INDICES2 mStackArrayDimIJ;		//Dimension of the tile array
	std::vector<bool> mIsBrightMap;

	bool isAvgBright_(const double threshold, const int II, const int JJ) const;
	bool isQuadrantBright_(const double threshold, const int II, const int JJ) const;
};

class QuickStitcher
{
public:
	QuickStitcher(const int tileWidth_pix, const int tileHeight_pix, const int nTileRow, const int nTileCol);
	void push(const TiffU8 &tile, const int II, const int JJ);
	void saveToFile(std::string filename, const OVERRIDE override) const;
private:
	TiffU8 mStitchedTiff;
	int mNrow;
	int mNcol;
	INDICES2 mStackArrayDimIJ;		//Dimension of the tile array
};