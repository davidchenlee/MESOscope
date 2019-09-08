#pragma once
#include "Utilities.h"
#include "Devices.h"
using namespace Constants;

double multiply16X(const double input);
void reverseSCANDIR(SCANDIR &scanDir);
POSITION2 determineRelativeTileIndicesIJ(const TILEOVERLAP3 overlapIJK_frac, const INDICES2 tileArraySizeIJ, const INDICES2 tileIndicesIJ);
double determineInitialScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineFinalScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir);
double determineInitialLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);
double determineFinalLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDir);
std::string convertWavelengthToFluorMarker_s(const int wavelength_nm);


class TileArray
{
public:
	enum Axis { II, JJ, KK };		//Currently, the tile axis II coincides with Stage::Axis::XX, JJ with Stage::Axis::YY, and KK with Stage::Axis::ZZ
	TileArray(const int tileHeight_pix, const int tileWidth_pix, const INDICES2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac);
	TileArray(const PIXELS2 tileSize_pix, const INDICES2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac);
	int readTileHeight_pix() const;
	int readTileWidth_pix() const;
	INDICES2 readTileArraySizeIJ() const;
	int readTileArraySizeIJ(const Axis axis) const;
	TILEOVERLAP3 readTileOverlapIJK_frac() const;
	PIXELS2 determineTileRelativePixelPos_pix(const INDICES2 tileIndicesIJ) const;
protected:
	const int mTileHeight_pix;		//Pixel height of a single tile
	const int mTileWidth_pix;		//Pixel width of a single tile
	INDICES2 mArraySizeIJ;			//Dimension of the array of tiles
	TILEOVERLAP3 mOverlapIJK_frac;
};

class QuickStitcher : public TiffU8, public TileArray
{
public:
	QuickStitcher(const int tileHeight_pix, const int tileWidth_pix, const INDICES2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac);
	void push(const U8 *tile, const INDICES2 tileIndicesIJ);
	void saveToFile(std::string filename, const OVERRIDE override) const;
	int readFullHeight_pix() const;
	int readFullWidth_pix() const;
private:
	int mFullHeight;
	int mFullWidth;
};

class QuickScanXY final: public QuickStitcher
{
public:
	QuickScanXY(const POSITION2 ROIcenterXY, const FFOV2 ffov, const SIZE2 pixelSizeXY, const SIZE2 LOIxy);
	double determineInitialScanPosX(const double travelOverhead, const SCANDIR scanDir) const;
	double determineFinalScanPosX(const double travelOverhead, const SCANDIR scanDir) const;
	int readNstageYpos() const;
	double readStageYposFront() const;
	double readStageYposBack() const;
	double readStageYposAt(const int index) const;
private:
	std::vector<double> mStageYpos;
	const POSITION2 mROIcenterXY;
	const FFOV2 mFFOV;
	const SIZE2 mPixelSizeXY;
	const SIZE2 mLOIxy;
	const int mFullWidth_pix;

	int castToOddnumber_(const double input) const;
	SIZE2 castLOIxy_(const FFOV2 FFOV, const SIZE2 LOIxy) const;
};

class Boolmap final
{
public:
	Boolmap(const TiffU8 &tiff, const PIXELS2 tileSize_pix, const TILEOVERLAP3 overlapIJK_frac, const double threshold);
	Boolmap(const QuickScanXY &quickScanXY, const PIXELS2 tileSize_pix, const TILEOVERLAP3 overlapIJK_frac, const double threshold);
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

class FluorMarkerList	//Create a list of fluorescent markers
{
public:
	struct FluorMarker //Parameters for a single fluorescent marker
	{
		std::string mName{ "" };	//Fluorescent marker name
		int mWavelength_nm;			//Laser wavelength
		double mScanPmin;			//Initial laser power for a stack scan. It could be >= or <= than the final laser power depending on the scan direction
		double mScanPexp;			//Length constant for the exponential power increase
		int nFramesBinning{ 1 };
	};

	FluorMarkerList(const std::vector<FluorMarker> fluorMarkerList);
	std::size_t readFluorMarkerListSize() const;
	void printFluorParams(std::ofstream *fileHandle) const;
	FluorMarker findFluorMarker(const std::string fluorMarker) const;
	FluorMarker readFluorMarker(const int indexFluorMarker) const;
	FluorMarkerList readFluorMarkerList() const;
private:
	std::vector<FluorMarker> mFluorMarkerList;
};

class Sample : public FluorMarkerList
{
public:
	 //Initialize with std::numeric_limits<double>::quiet_NaN??
	POSITION2 mCenterXY{-1, -1};						//Sample center (stageX, stageY)
	SIZE3 mLOIxyz_req{ -1, -1, -1 };					//Requested Length of interest (stageX, stageY, stageZ)
	double mSurfaceZ{ -1 };
	const double mBladeFocalplaneOffsetZ{ 1.06 * mm };	//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise
	double mCutAboveBottomOfStack{ 0. * um };			//Specify at what height of the overlapping volume to cut

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorMarkerList fluorMarkerList = { {} });
	Sample(const Sample& sample, const POSITION2 centerXY, const SIZE3 LOIxyz, const double sampleSurfaceZ, const double sliceOffset);
	void printSampleParams(std::ofstream *fileHandle) const;
	std::string readName() const;
	std::string readImmersionMedium() const;
	std::string readObjectiveCollar() const;
	std::vector<LIMIT2> readStageSoftPosLimXYZ() const;
	double readStageSoftPosLimXMIN() const;
	double readStageSoftPosLimXMAX() const;
	double readStageSoftPosLimYMIN() const;
	double readStageSoftPosLimYMAX() const;
private:
	std::string mName;
	std::string mImmersionMedium;
	std::string mObjectiveCollar;	
	std::vector<LIMIT2> mStageSoftPosLimXYZ;			//Soft position limits of the stages
};

class Stack final
{
public:
	Stack(const FFOV2 FFOV, const int tileHeight_pix, int const tileWidth_pix, const double pixelSizeZ, const int nFrames, const TILEOVERLAP3 overlapIJK_frac);
	void printParams(std::ofstream *fileHandle) const;
	double readFFOV(const Stage::Axis axis) const;
	int readTileHeight_pix() const;
	int readTileWidth_pix() const;
	double readPixelSizeZ() const;
	double readDepthZ() const;
	TILEOVERLAP3 readOverlapIJK_frac() const;
	double readOverlap_frac(const TileArray::Axis axis) const;
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
	class MoveStage final
	{
	public:
		void setParam(const int sliceNumber, const INDICES2 tileIndicesIJ, const POSITION2 tileCenterXY);
		int readSliceNumber() const;
		int readTileIndex(const TileArray::Axis axis) const;
		POSITION2 readTileCenterXY() const;
		double readTileCenter(Stage::Axis axis) const;
	private:
		int mSliceNumber;			//Slice number
		INDICES2 mTileIndicesIJ;	//Indices of the tile array
		POSITION2 mTileCenterXY;	//X-stage and Y-stage positions corresponding to the center of the tile
	};

	class AcqStack final
	{
	public:
		void setParam(const int stackNumber, const int wavelength_nm, const SCANDIR scanDirZ, const double scanZmin, const double depthZ, const double scanPmin, const double scanPexp, const int nFrameBinning);
		int readStackNumber() const;
		int readWavelength_nm() const;
		SCANDIR readScanDirZ() const;
		double readScanZmin() const;
		double readDepthZ() const;
		double readScanPmin() const;
		double readScanPexp() const;
		int readNframeBinning() const;
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

	class CutSlice final
	{
	public:
		void setParam(const double planeZtoCut, const double stageZheightForFacingTheBlade);
		double readPlaneZtoCut() const;
		double readStageZheightForFacingTheBlade() const;
	private:
		double mPlaneZtoCut;
		double mStageZheightForFacingTheBlade;
	};
}

//A list of commands that form a full sequence
class Sequencer final 
{
public:
	class Commandline final	//Single commands
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
	INDICES2 determineTileArraySizeIJ_();
	void initializeEffectiveROI_();
	void reserveMemoryBlock_();
	void initializeIteratorIJ_();
	void resetStageScanDirections_();
	SIZE3 determineEffectiveLOIxyz() const;

	void moveStage_(const INDICES2 tileIndicesIJ);
	void acqStack_(const int indexFluorMarker);
	void saveStack_();
	void cutSlice_();
};