#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

double multiply16X(const double input);
void reverseSCANDIR(SCANDIR &scanDir);
double determineInitialScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineFinalScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineInitialLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);
double determineFinalLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);

struct FluorLabelList	//Create a list of fluorescent labels
{
	struct FluorLabel //Parameters for a single fluorescent label
	{
		std::string mName{ "" };	//Fluorescent label name
		int mWavelength_nm;			//Laser wavelength
		double mScanPmin;			//Initial laser power for a stack scan. It could be >= or <= than the final laser power depending on the scan direction
		double mScanPinc;			//Laser power increase per unit of distance in the Z-stage axis
		int nFramesBinning{ 1 };
	};
	std::vector<FluorLabel> mFluorLabelList;
	FluorLabelList(const std::vector<FluorLabel> fluorLabelList);
	std::size_t size() const;
	FluorLabel front() const;
	FluorLabel at(const int index) const;
	void printParams(std::ofstream *fileHandle) const;
	FluorLabel findFluorLabel(const std::string fluorLabel) const;
};

struct Sample
{
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;	
	POSITION2 mCenterXY;									//Sample center (stageX, stageY)
	SIZE3 mSampleSizeXYZreq{ 0, 0, 0 };						//Requested sample size (stageX, stageY, stageZ)
	double mSurfaceZ;
	FluorLabelList mFluorLabelList;
	std::vector<LIMIT2> mStageSoftPosLimXYZ;				//Soft position limits of the stages

	const POSITION2 mBladePositionXY{ 0. * mm, 0. * mm };	//Location of the vibratome blade in the X-stage and Y-stage axes wrt the stages origin
	const double mBladeFocalplaneOffsetZ{ 0. * um };		//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack{ 0. * um };				//Specify at what height of the overlapping volume to cut

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList = { {} });
	Sample(const Sample& sample, const POSITION2 centerXY, const SIZE3 sampleSizeXYZ, const double sampleSurfaceZ, const double sliceOffset);
	FluorLabelList::FluorLabel findFluorLabel(const std::string fluorLabel) const;
	void printParams(std::ofstream *fileHandle) const;
};

struct Stack
{
	FFOV2 mFFOV;					//Full field of view in the X-stage and Y-stage axes
	int mTileHeight_pix;
	int mTileWidth_pix;
	double mPixelSizeZ;				//Image resolution in the Z-stage axis
	double mDepthZ;					//Stack depth or thickness
	TILEOVERLAP3 mOverlapXYZ_frac;	//Stack overlap in the X-stage, Y-stage, and Z-stage axes

	Stack(const FFOV2 FFOV, const int tileHeight_pix, int const tileWidth_pix, const double pixelSizeZ, const int nFrames, const TILEOVERLAP3 overlapXYZ_frac);
	void printParams(std::ofstream *fileHandle) const;
};

namespace Action
{
	enum class ID { CUT, ACQ, SAV, MOV };
	struct MoveStage {
		int mSliceNumber;			//Slice number
		INDICES2 mTilesIJ;			//Indices of the tile array
		POSITION2 mTileCenterXY;	//X-stage and Y-stage positions corresponding to the center of the tile
	};
	struct AcqStack {
		int mStackNumber;
		int mWavelength_nm;
		SCANDIR mScanDirZ;		//THIS IS NOT READ BY THE SEQUENCER ANYMORE!!
		double mScanZmin;		//Min z position of a stack scan
		double mDepthZ;			//Stack depth (thickness)
		double mScanPmin;		//Min laser power of the stack scan (at the top of the stack)
		double mScanPinc;		//Laser power increase in the Z-stage axis per unit of distance
		int nFrameBinning;
	};
	struct CutSlice {
		POSITION3 mBladePositionXY;		//Position the sample facing the vibratome blade
	};
}

class TileArray
{
public:
	int mTileHeight_pix;			//Pixel height of a single tile
	int mTileWidth_pix;				//Pixel width of a single tile
	int mNpix;						//Total number of pixels in a single tile
	INDICES2 mArraySize;			//Dimension of the array of tiles

	TileArray(const int tileHeight_pix, const int tileWidth_pix, const INDICES2 tileArraySize, const TILEOVERLAP3 overlapXYZ_frac);
	PIXELS2 determineRelativeTilePosition_pix(const INDICES2 tileIndicesIJ) const;
	void TileArray::asd(const POSITION2 centerPosition, const FFOV2 ffo) const;
private:
	std::vector<INDICES2> mTilePosition_pix;
	TILEOVERLAP3 mOverlapXYZ_frac;
};

class BoolMap
{
public:
	BoolMap(const TiffU8 &tiff, const TileArray tileArray, const double threshold);
	bool isTileBright(const INDICES2 tileIndicesIJ);
	void saveTileMapToText(std::string filename);

	void SaveTileGridOverlap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
	void saveTileMap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
private:
	const TiffU8 &mTiff;
	double mThreshold;						//Threshold for generating the boolmap
	int mHeight_pix;						//Pixel height of the tiled image
	int mWidth_pix;							//Pixel width of the tiled image
	int mNpix;								//Total number of pixels in mTiff
	TileArray mTileArray;
	PIXELS2 mTileArrayCenter_pix;			//Reference position for the tile array wrt the Tiff

	std::vector<bool> mIsBrightMap;

	PIXELS2 determineAbsoluteTilePosition_pix_(const INDICES2 tileIndicesIJ) const;
	bool isAvgBright_(const double threshold, const INDICES2 tileIndicesIJ) const;
	bool isQuadrantBright_(const double threshold, const INDICES2 tileIndicesIJ) const;
};

class Sequencer	//A list of commands that forms a full sequence
{
public:
	class Commandline	//Single commands
	{
	public:
		Action::ID mAction;
		union {
			Action::MoveStage moveStage;
			Action::AcqStack acqStack;
			Action::CutSlice cutSlice;
		} mParam;

		Commandline(const Action::ID action);

		std::string printHeader() const;
		std::string printHeaderUnits() const;
		void printToFile(std::ofstream *fileHandle) const;
		void printParameters() const;
	private:
		std::string actionToString_(const Action::ID action) const;
	};
	Sequencer(const Sample sample, const Stack stack);
	Sequencer(const Sequencer&) = delete;					//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;		//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;						//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;				//Disable move-assignment constructor

	Commandline readCommandline(const int iterCommandline) const;
	void generateCommandList();
	//std::vector<POSITION2> generateLocationList();
	int size() const;
	POSITION2 tileIndicesIJToStagePositionXY(const INDICES2 tileIndicesIJ) const;
	void printSequenceParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
private:
	Sample mSample;											//Sample
	const Stack mStack;										//Stack
	std::vector<Commandline> mCommandList;
	int mII;												//Tile iterator for the X-stage
	int mJJ;												//Tile iterator for the Y-stage
	int mCommandCounter{ 0 };
	ROI4 mROIeff;											//Not used in the sequencer. It is just for printing out the ROI
	int mStackCounter{ 0 };									//Count the number of stacks
	int mSliceCounter{ 0 };									//Count the number of the slices
	TileArray mTileArray;
	SCANDIR3 mIterScanDirXYZ{ mInitialScanDirXYZ };			//Scan directions wrt the X-stage, Y-stage, and Z-stage axes	
	double mScanZi;											//Initial Z-stage position for a stack scan
	double mPlaneToSliceZ{ 0 };								//Height of the plane to cut	
	int mNtotalSlices;										//Number of vibratome slices in the entire sample
	const SCANDIR3 mInitialScanDirXYZ{ SCANDIR::LEFTWARD,	//Initial scan directions wrt the X-stage, Y-stage, and Z-stage axes (the imaging has the opposite direction)
										SCANDIR::OUTWARD,
										SCANDIR::UPWARD };

	void initializeVibratomeSlice_();
	INDICES2 determineTileArraySize_();
	void initializeEffectiveROI_();
	void reserveMemoryBlock_();
	void initializeIteratorIJ_();
	void resetStageScanDirections_();
	SIZE3 effectiveSampleSizeXYZ_() const;

	void moveStage_(const INDICES2 tileIndicesIJ);
	void acqStack_(const int wavelengthIndex);
	void saveStack_();
	void cutSlice_();
};