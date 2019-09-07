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
std::string indexFluorLabel_s(const int wavelength_nm);

class FluorLabelList	//Create a list of fluorescent labels
{
public:
	struct FluorLabel //Parameters for a single fluorescent label
	{
		std::string mName{ "" };	//Fluorescent label name
		int mWavelength_nm;			//Laser wavelength
		double mScanPmin;			//Initial laser power for a stack scan. It could be >= or <= than the final laser power depending on the scan direction
		double mScanPexp;			//Length constant for the exponential power increase
		int nFramesBinning{ 1 };
	};

	FluorLabelList(const std::vector<FluorLabel> fluorLabelList);
	std::size_t readNtotalFluorLabels() const;
	FluorLabel front() const;
	FluorLabel at(const int index) const;
	void printFluorParams(std::ofstream *fileHandle) const;
	FluorLabel findFluorLabel(const std::string fluorLabel) const;
	FluorLabelList readFluorLabelList() const { return mFluorLabelList; }
	FluorLabel  readFluorLabel(const int wavelengthIndex) const { return mFluorLabelList.at(wavelengthIndex); }
private:
	std::vector<FluorLabel> mFluorLabelList;
};

class Sample : public FluorLabelList
{
public:
	 //Initialize with std::numeric_limits<double>::quiet_NaN??
	POSITION2 mCenterXY{-1, -1};						//Sample center (stageX, stageY)
	SIZE3 mLOIxyz_req{ -1, -1, -1 };					//Requested Length of interest (stageX, stageY, stageZ)
	double mSurfaceZ{ -1 };
	const double mBladeFocalplaneOffsetZ{ 1.06 * mm };	//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack{ 0. * um };			//Specify at what height of the overlapping volume to cut

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList = { {} });
	Sample(const Sample& sample, const POSITION2 centerXY, const SIZE3 LOIxyz, const double sampleSurfaceZ, const double sliceOffset);
	void printSampleParams(std::ofstream *fileHandle) const;

	std::string readName() const { return mName; }
	std::string readImmersionMedium() const { return mImmersionMedium; }
	std::string readObjectiveCollar() const { return mObjectiveCollar; }
	std::vector<LIMIT2> readStageSoftPosLimXYZ() const { return mStageSoftPosLimXYZ; }
	double readStageSoftPosLimXMIN() const { return mStageSoftPosLimXYZ.at(Stage::Axis::XX).MIN; }
	double readStageSoftPosLimXMAX() const { return mStageSoftPosLimXYZ.at(Stage::Axis::XX).MAX; }
	double readStageSoftPosLimYMIN() const { return mStageSoftPosLimXYZ.at(Stage::Axis::YY).MIN; }
	double readStageSoftPosLimYMAX() const { return mStageSoftPosLimXYZ.at(Stage::Axis::YY).MAX; }
private:
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;	
	std::vector<LIMIT2> mStageSoftPosLimXYZ;			//Soft position limits of the stages
};

class Stack
{
public:
	Stack(const FFOV2 FFOV, const int tileHeight_pix, int const tileWidth_pix, const double pixelSizeZ, const int nFrames, const TILEOVERLAP3 overlapIJK_frac);
	void printParams(std::ofstream *fileHandle) const;
	double readFFOV(const Stage::Axis axis) const
	{
		switch (axis)
		{
		case Stage::Axis::XX:
			return mFFOV.XX;
		case Stage::Axis::YY:
			return mFFOV.YY;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected stage axis unavailable");
		}
	}
	int readTileHeight_pix() const { return mTileHeight_pix; }
	int readTileWidth_pix() const { return mTileWidth_pix; }
	double readPixelSizeZ() const { return mPixelSizeZ;  }
	double readDepthZ() const { return mDepthZ; }
	TILEOVERLAP3 readOverlapIJK_frac() const { return mOverlapIJK_frac; }
	double readOverlap_frac(const TileArray::Axis axis) const
	{
		switch (axis)
		{
		case TileArray::Axis::II:
			return mOverlapIJK_frac.II;
		case TileArray::Axis::JJ:
			return mOverlapIJK_frac.JJ;
		case TileArray::Axis::KK:
			return mOverlapIJK_frac.KK;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected tile array axis unavailable");
		}
	}
private:
	FFOV2 mFFOV;					//Full field of view in the X-stage and Y-stage axes
	int mTileHeight_pix;
	int mTileWidth_pix;
	double mPixelSizeZ;				//Image resolution in the Z-stage axis
	double mDepthZ;					//Stack depth or thickness
	TILEOVERLAP3 mOverlapIJK_frac;	//Stack overlap in the X-stage, Y-stage, and Z-stage axes
};

namespace Action
{
	enum class ID { CUT, ACQ, SAV, MOV };
	class MoveStage
	{
	public:
		void setParam(const int sliceNumber, const INDICES2 tileIndicesIJ, const POSITION2 tileCenterXY)
		{
			mSliceNumber = sliceNumber;
			mTileIndicesIJ = tileIndicesIJ;
			mTileCenterXY = tileCenterXY;
		}
		int readSliceNumber() const { return mSliceNumber; }
		int readTileIndex(const TileArray::Axis axis) const
		{
			switch (axis)
			{
			case TileArray::Axis::II:
				return mTileIndicesIJ.II;
			case TileArray::Axis::JJ:
				return mTileIndicesIJ.JJ;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected tile array axis unavailable");
			}
		}

		POSITION2 readTileCenterXY() const { return mTileCenterXY; }
		double readTileCenter(Stage::Axis axis) const
		{
			switch (axis)
			{
			case Stage::Axis::XX:
				return mTileCenterXY.XX;
			case Stage::Axis::YY:
				return mTileCenterXY.YY;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected stage axis unavailable");
			}
		}
	private:
		int mSliceNumber;			//Slice number
		INDICES2 mTileIndicesIJ;	//Indices of the tile array
		POSITION2 mTileCenterXY;	//X-stage and Y-stage positions corresponding to the center of the tile
	};

	class AcqStack
	{
	public:
		void setParam(const int stackNumber, const int wavelength_nm, const SCANDIR scanDirZ, const double scanZmin, const double depthZ, const double scanPmin, const double scanPexp, const int nFrameBinning)
		{
			mStackNumber = stackNumber;
			mWavelength_nm = wavelength_nm;
			mScanDirZ = scanDirZ;
			mScanZmin = scanZmin;
			mDepthZ = depthZ;
			mScanPmin = scanPmin;
			mScanPexp = scanPexp;
			mNframeBinning = nFrameBinning;
		}
		int readStackNumber() const { return mStackNumber; }
		int readWavelength_nm() const { return mWavelength_nm; }
		SCANDIR readScanDirZ() const { return mScanDirZ; }
		double readScanZmin() const { return mScanZmin; }
		double readDepthZ() const { return mDepthZ; }
		double readScanPmin() const { return mScanPmin; }
		double readScanPexp() const { return mScanPexp; }
		int readNframeBinning() const { return mNframeBinning; }
	private:
		int mStackNumber;
		int mWavelength_nm;
		SCANDIR mScanDirZ;		//THIS IS NOT READ BY THE SEQUENCER ANYMORE!!
		double mScanZmin;		//Min z position of a stack scan
		double mDepthZ;			//Stack depth (thickness)
		double mScanPmin;		//Min laser power of the stack scan (at the top of the stack)
		double mScanPexp;		//Laser power increase in the Z-stage axis per unit of distance
		int mNframeBinning;
	};

	class CutSlice
	{
	public:
		void setParam(const double planeZtoCut, const double stageZheightForFacingTheBlade)
		{
			mPlaneZtoCut = planeZtoCut;
			mStageZheightForFacingTheBlade = stageZheightForFacingTheBlade;
		}
		double readPlaneZtoCut() const { return mPlaneZtoCut; }
		double readStageZheightForFacingTheBlade() const { return mStageZheightForFacingTheBlade; }
	private:
		double mPlaneZtoCut;
		double mStageZheightForFacingTheBlade;
	};
}

class QuickScanXY: public QuickStitcher
{
public:
	std::vector<double> mStagePosY;

	QuickScanXY(const POSITION2 ROIcenterXY, const FFOV2 ffov, const SIZE2 pixelSizeXY, const SIZE2 LOIxy);
	double determineInitialScanPosX(const double travelOverhead, const SCANDIR scanDir) const;
	double determineFinalScanPosX(const double travelOverhead, const SCANDIR scanDir) const;
private:
	const POSITION2 mROIcenterXY;
	const FFOV2 mFFOV;
	const SIZE2 mPixelSizeXY;
	const SIZE2 mLOIxy;
	const int mFullWidth_pix;
};

class Boolmap
{
public:
	Boolmap(const TiffU8 &tiff, const TileArray tileArray, const PIXELS2 anchorPixel_pix, const double threshold);
	bool isTileBright(const INDICES2 tileIndicesIJ) const;
	void saveTileMapToText(std::string filename);
	void saveTileGridOverlap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
	void saveTileMap(std::string filename, const OVERRIDE override = OVERRIDE::DIS) const;
private:
	const TiffU8 &mTiff;
	const TileArray mTileArray;
	const double mThreshold;			//Threshold for generating the boolmap
	const int mFullHeight_pix;			//Pixel height of the tiled image
	const int mFullWidth_pix;			//Pixel width of the tiled image
	const int mNpixFull;				//Total number of pixels in mTiff
	PIXELS2 mAnchorPixel_pix;			//Reference position for the tile array wrt the Tiff
	std::vector<bool> mIsBrightMap;

	PIXELS2 determineTileAbsolutePixelPos_pix_(const INDICES2 tileIndicesIJ) const;
	bool isQuadrantBright_(const double threshold, const INDICES2 tileIndicesIJ) const;
};

class Sequencer	//A list of commands that form a full sequence
{
public:
	class Commandline	//Single commands
	{
	public:
		Action::ID mActionID;
		union {
			Action::MoveStage moveStage;
			Action::AcqStack acqStack;
			Action::CutSlice cutSlice;
		} mAction;

		Commandline(const Action::ID actionID);
		void printToFile(std::ofstream *fileHandle) const;
		void printParameters() const;
	private:
		std::string convertActionIDtoString_(const Action::ID actionID) const;
	};
	Sequencer(const Sample sample, const Stack stack);
	Sequencer(const Sequencer&) = delete;					//Disable copy-constructor
	Sequencer& operator=(const Sequencer&) = delete;		//Disable assignment-constructor
	Sequencer(Sequencer&&) = delete;						//Disable move constructor
	Sequencer& operator=(Sequencer&&) = delete;				//Disable move-assignment constructor

	void generateCommandList();
	POSITION2 convertTileIndicesIJToStagePosXY(const INDICES2 tileIndicesIJ) const;
	std::string printHeader() const;
	std::string printHeaderUnits() const;
	void printSequenceParams(std::ofstream *fileHandle) const;
	void printToFile(const std::string fileName) const;
	int readNtotalSlices() const;
	int readNtotalStacks() const;
	int readNtotalCommands() const;
	Commandline readCommandline(const int iterCommandline) const;
private:
	const Sample mSample;									//Sample
	const Stack mStack;										//Stack
	std::vector<Commandline> mCommandList;
	int mII{ 0 };											//Tile iterator for the X-stage
	int mJJ{ 0 };											//Tile iterator for the Y-stage
	int mCommandCounter{ 0 };
	ROI4 mROI;												//Effective ROI covered by the tile array. It could be slightly larger than the size specified by mSample.mLOIxyz_req
	int mStackCounter{ 0 };									//Count the number of stacks
	int mSliceCounter{ 0 };									//Count the number of the slices
	TileArray mTileArray;
	SCANDIR3 mIterScanDirXYZ{ g_initialStageScanDirXYZ };	//Scan directions wrt the X-stage, Y-stage, and Z-stage axes	
	double mIterScanZi;										//Initial Z-stage position for a stack scan
	double mIterSamplePlaneZtoCut;							//Sample plane to cut (height of the stage Z)
	double mIterStageZheightForFacingTheBlade;				//Actual height of the stage Z for cutting the sample at mIterSamplePlaneZtoCut
															//(It defers from mIterSamplePlaneZtoCut by the height offset of the blade wrt the imaging plane)
	int mNtotalSlices;										//Number of vibratome slices in the entire sample

	void initializeVibratomeSlice_();
	INDICES2 determineTileArraySize_();
	void initializeEffectiveROI_();
	void reserveMemoryBlock_();
	void initializeIteratorIJ_();
	void resetStageScanDirections_();
	SIZE3 determineEffectiveLOIxyz() const;

	void moveStage_(const INDICES2 tileIndicesIJ);
	void acqStack_(const int wavelengthIndex);
	void saveStack_();
	void cutSlice_();
};