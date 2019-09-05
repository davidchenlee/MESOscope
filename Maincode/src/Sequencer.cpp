#include "Sequencer.h"

//Used to increase the laser power 16 times
double multiply16X(const double input)
{
	return 16 * input;
}

//Switch the scan direction. It is important to pass scanDir by reference to be able to modify it
void reverseSCANDIR(SCANDIR &scanDir)
{
	switch (scanDir)
	{
		//X-stage
	case SCANDIR::LEFTWARD:
		scanDir = SCANDIR::RIGHTWARD;
		break;
	case SCANDIR::RIGHTWARD:
		scanDir = SCANDIR::LEFTWARD;
		break;
		//Y-stage
	case SCANDIR::OUTWARD:
		scanDir = SCANDIR::INWARD;
		break;
	case SCANDIR::INWARD:
		scanDir = SCANDIR::OUTWARD;
		break;
		//Z-stage
	case SCANDIR::DOWNWARD:
		scanDir = SCANDIR::UPWARD;
		break;
	case SCANDIR::UPWARD:
		scanDir = SCANDIR::DOWNWARD;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

double determineInitialScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The travel range must be > 0");

	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin - travelOverhead;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin + travel + travelOverhead;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

//Travel overhead to avoid the nonlinearity at the end of the stage scan
double determineFinalScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The travel range must be > 0");

	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin + travel + travelOverhead;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin - travelOverhead;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

//The initial laser power for scanning a stack depends on whether the stack is imaged from the top down or from the bottom up
//totalPowerInc is the total increase and NOT the increase per unit of length
double determineInitialLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0 || totalPowerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The laser power and power increase must be >= 0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin;
	case SCANDIR::DOWNWARD:
		return powerMin + totalPowerInc;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

//totalPowerInc is the total increase and NOT the increase per unit of length
double determineFinalLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0 || totalPowerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The laser power and power increase must be >= 0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin + totalPowerInc;
	case SCANDIR::DOWNWARD:
		return powerMin;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

#pragma region "FluorLabelList"
FluorLabelList::FluorLabelList(const std::vector<FluorLabel> fluorLabelList) :
	mFluorLabelList{ fluorLabelList }
{}

std::size_t FluorLabelList::size() const
{
	return mFluorLabelList.size();
}

FluorLabelList::FluorLabel FluorLabelList::front() const
{
	return mFluorLabelList.front();
}

FluorLabelList::FluorLabel FluorLabelList::at(const int index) const
{
	return mFluorLabelList.at(index);
}

void FluorLabelList::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "LASERS ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mFluorLabelList.size(); iterWL++)
	{
		*fileHandle << "Wavelength = " << mFluorLabelList.at(iterWL).mWavelength_nm <<
			" nm\nPower = " << mFluorLabelList.at(iterWL).mScanPmin / mW <<
			" mW\nPower exponential length = " << mFluorLabelList.at(iterWL).mScanPexp / um << " um\n";
	}
	*fileHandle << "\n";
}

//Return the first instance of "fluorLabel" in mFluorLabelList
FluorLabelList::FluorLabel FluorLabelList::findFluorLabel(const std::string fluorLabel) const
{
	for (std::vector<int>::size_type iter_label = 0; iter_label < mFluorLabelList.size(); iter_label++)
	{
		if (!fluorLabel.compare(mFluorLabelList.at(iter_label).mName)) //compare() returns 0 if the strings are identical
			return mFluorLabelList.at(iter_label);
	}
	//If the requested fluorLabel is not found
	throw std::runtime_error((std::string)__FUNCTION__ + ": Fluorescent label " + fluorLabel + " not found");
}
#pragma endregion "FluorLabelList"

#pragma region "Sample"
Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorLabelList fluorLabelList) :
	mName{ sampleName },
	mImmersionMedium{ immersionMedium },
	mObjectiveCollar{ objectiveCollar },
	mStageSoftPosLimXYZ{ stageSoftPosLimXYZ },
	mFluorLabelList{ fluorLabelList }
{}

Sample::Sample(const Sample& sample, const POSITION2 centerXY, const SIZE3 LOIxyz, const double sampleSurfaceZ, const double sliceOffset) :
	mName{ sample.mName },
	mImmersionMedium{ sample.mImmersionMedium },
	mObjectiveCollar{ sample.mObjectiveCollar },
	mFluorLabelList{ sample.mFluorLabelList },
	mCenterXY{ centerXY },
	mLOIxyz_req{ LOIxyz },
	mSurfaceZ{ sampleSurfaceZ },
	mCutAboveBottomOfStack{ sliceOffset }
{
	if (mLOIxyz_req.XX <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length X must be > 0");
	if (mLOIxyz_req.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The sample length Y must be > 0");
	if (mCutAboveBottomOfStack < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice offset must be >= 0");
}

FluorLabelList::FluorLabel Sample::findFluorLabel(const std::string fluorLabel) const
{
	return mFluorLabelList.findFluorLabel(fluorLabel);
}

void Sample::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SAMPLE ************************************************************\n";
	*fileHandle << "Name = " << mName << "\n";
	*fileHandle << "Immersion medium = " << mImmersionMedium << "\n";
	*fileHandle << "Correction collar = " << mObjectiveCollar << "\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Sample center (stageX, stageY) = (" << mCenterXY.XX / mm << " mm, " << mCenterXY.YY / mm << " mm)\n";
	*fileHandle << "Requested ROI size (stageX, stageY, stageZ) = (" << mLOIxyz_req.XX / mm << " mm, " << mLOIxyz_req.YY / mm << " mm, " << mLOIxyz_req.ZZ / mm << " mm)\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Blade position (stageX, stageY) = (" << mBladePosXY.XX / mm << " mm, " << mBladePosXY.YY / mm << " mm)\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset = " << mBladeFocalplaneOffsetZ / um << " um\n";
	*fileHandle << "Cut above the bottom of the stack = " << mCutAboveBottomOfStack / um << " um\n";
	*fileHandle << "\n";
}
#pragma endregion "Sample"

#pragma region "Stack"
Stack::Stack(const FFOV2 FFOV, const int tileHeight_pix, int const tileWidth_pix, const double pixelSizeZ, const int nFrames, const TILEOVERLAP3 overlapXYZ_frac) :
	mFFOV{ FFOV },
	mTileHeight_pix{ tileHeight_pix },
	mTileWidth_pix{ tileWidth_pix },
	mPixelSizeZ{ pixelSizeZ },
	mDepthZ{ pixelSizeZ *  nFrames },
	mOverlapXYZ_frac{ overlapXYZ_frac }
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be positive");
	if (tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile height must be > 0");
	if (tileWidth_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile width must be > 0");
	if (mPixelSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel size Z must be > 0");
	if (mDepthZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack depth Z must be positive");
	if (mOverlapXYZ_frac.XX < 0 || mOverlapXYZ_frac.YY < 0 || mOverlapXYZ_frac.XX > 0.2 || mOverlapXYZ_frac.YY > 0.2 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in XY must be in the range [0-0.2]");
	if (mOverlapXYZ_frac.ZZ < 0 || mOverlapXYZ_frac.ZZ > 0.5)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in Z must be in the range [0-0.5]");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (stageX, stageY) = (" << mFFOV.XX / um << " um, " << mFFOV.YY / um << " um)\n";
	*fileHandle << "Pixel size Z = " << mPixelSizeZ / um << " um\n";
	*fileHandle << "Stack depth Z= " << mDepthZ / um << " um\n";
	*fileHandle << std::setprecision(2);
	*fileHandle << "Stack overlap = (" << mOverlapXYZ_frac.XX << ", " << mOverlapXYZ_frac.YY << ", " << mOverlapXYZ_frac.ZZ << ") (frac)\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Stack overlap = (" << mOverlapXYZ_frac.XX * mFFOV.XX / um << " um, " << mOverlapXYZ_frac.YY * mFFOV.YY / um << " um, " << mOverlapXYZ_frac.ZZ * mDepthZ << " um)\n";
	*fileHandle << "\n";
}
#pragma endregion "Stack"

#pragma region "Commandline"
Sequencer::Commandline::Commandline(const Action::ID action) :
	mAction{ action }
{}

std::string Sequencer::Commandline::printHeader() const
{
	return 	"Action\tSlice#\tTileIJ\t(stageX,stageY)\tStack#\tWavlen\tDirZ\tstageZ<\tstageZ>\tPmin\tPexp";
}

std::string Sequencer::Commandline::printHeaderUnits() const
{
	return "\t\t(mm,mm)\t\t\tnm\t\tmm\tmm\tmW\tmW";
}

void Sequencer::Commandline::printToFile(std::ofstream *fileHandle) const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		*fileHandle << actionToString_(mAction) << "\t" << mParam.moveStage.mSliceNumber;
		*fileHandle << "\t(" << mParam.moveStage.mTileIJ.II << "," << mParam.moveStage.mTileIJ.JJ << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mParam.moveStage.mTileCenterXY.XX / mm << "," << mParam.moveStage.mTileCenterXY.YY / mm << ")\n";
		break;
	case Action::ID::ACQ:
		*fileHandle << actionToString_(mAction) << "\t\t\t\t\t";
		*fileHandle << mParam.acqStack.mStackNumber << "\t";
		*fileHandle << mParam.acqStack.mWavelength_nm << "\t";
		*fileHandle << SCANDIRtoInt(mParam.acqStack.mScanDirZ) << "\t";
		*fileHandle << std::setprecision(3);
		*fileHandle << mParam.acqStack.mScanZmin / mm << "\t";
		*fileHandle << (mParam.acqStack.mScanZmin + mParam.acqStack.mDepthZ) / mm << "\t";
		*fileHandle << std::setprecision(0);
		*fileHandle << mParam.acqStack.mScanPmin << "\t" << mParam.acqStack.mScanPexp << "\n";
		break;
	case Action::ID::SAV:
		*fileHandle << actionToString_(mAction) + "\n";
		break;
	case Action::ID::CUT:
		*fileHandle << actionToString_(mAction);
		*fileHandle << std::setprecision(3);
		*fileHandle << "\t************Sample facing the vibratome at = ";
		*fileHandle << "(" << mParam.cutSlice.mBladePosXY.XX / mm << "," << mParam.cutSlice.mBladePosXY.YY / mm << "," << mParam.cutSlice.mBladePosXY.ZZ / mm << ") mm";
		*fileHandle << "**************************************************************\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

void Sequencer::Commandline::printParameters() const
{
	switch (mAction)
	{
	case Action::ID::MOV:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mParam.moveStage.mSliceNumber << "\n";
		std::cout << "Tile ij = (" << mParam.moveStage.mTileIJ.II << "," << mParam.moveStage.mTileIJ.JJ << ")\n";
		std::cout << "Tile center (stageX, stageY) = (" << mParam.moveStage.mTileCenterXY.XX / mm << "," << mParam.moveStage.mTileCenterXY.YY / mm << ") mm\n\n";
		break;
	case Action::ID::ACQ:
		std::cout << "The command is " << actionToString_(mAction) << " with parameters: \n";
		std::cout << "wavelength = " << mParam.acqStack.mWavelength_nm << " nm\n";
		std::cout << "scanDirZ = " << SCANDIRtoInt(mParam.acqStack.mScanDirZ) << "\n";
		std::cout << "scanZmin / depthZ = " << mParam.acqStack.mScanZmin / mm << " mm/" << mParam.acqStack.mDepthZ << " mm\n";
		std::cout << "scanPmin / ScanPexp = " << mParam.acqStack.mScanPmin / mW << " mW/" << mParam.acqStack.mScanPexp / um << " (um)\n\n";
		break;
	case Action::ID::SAV:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	case Action::ID::CUT:
		std::cout << "The command is " << actionToString_(mAction) << "\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}

std::string Sequencer::Commandline::actionToString_(const Action::ID action) const
{
	switch (action)
	{
	case Action::ID::CUT:
		return "CUT";
	case Action::ID::SAV:
		return "SAV";
	case Action::ID::ACQ:
		return "ACQ";
	case Action::ID::MOV:
		return "MOV";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "QuickScanXY"
//In X, 
//In Y, 
QuickScanXY::QuickScanXY(const POSITION2 ROIcenterXY, const FFOV2 ffov, const SIZE2 pixelSizeXY, const SIZE2 LOIxy) :
	mROIcenterXY{ ROIcenterXY },
	mFFOV{ ffov },
	mPixelSizeXY{ pixelSizeXY },
	mLOIxy{ LOIxy },																//The length of interest (LOI) is from tile edge to tile edge
	mFullWidth_pix{ static_cast<int>(std::ceil(LOIxy.YY / pixelSizeXY.YY)) },
	mQuickStitcher{ static_cast<int>(std::ceil(LOIxy.XX / pixelSizeXY.XX)),			//tileHeight_pix. The tile height is the same as the full height (because a tile is a vertical strip)
					static_cast<int>(std::ceil(ffov.YY / pixelSizeXY.YY)),			//tileWidth_pix
					{ 1, static_cast<int>(std::ceil(1. * LOIxy.YY / ffov.YY)) },	//tile array size = Only 1 row and many columns
					{ 0, 0, 0} }													//No overlap	
{
	const int II{ 0 };							//Only 1 row
	for (int JJ = 0; JJ < mQuickStitcher.tileArraySize().JJ; JJ++)
	{
		const POSITION2 relativeTileIndexIJ{ determineRelativeTileIndicesIJ(mQuickStitcher.tileOverlap_frac(), mQuickStitcher.tileArraySize(), { II, JJ }) };
		mStagePosY.push_back(mROIcenterXY.YY - mFFOV.YY * relativeTileIndexIJ.YY);		//for now, only stacking strips to the right is allowed (i.e. move the stage to the left)
	}
}

double QuickScanXY::determineInitialScanPosX(const double travelOverhead, const SCANDIR scanDir) const
{
	const double ROIminX{ mROIcenterXY.XX - mLOIxy.XX / 2 };	//Sample edge
	const double tilePosXmin{ ROIminX + mFFOV.XX / 2. };		//The first tile center is mFFOV.XX / 2 away from the ROI edge
	const double travelX{ mLOIxy.XX - mFFOV.XX };				//The X stage does not travel the first and last mFFOV.XX / 2 from the edge of the ROI

	return determineInitialScanPos(tilePosXmin, travelX, travelOverhead, scanDir);
}

double QuickScanXY::determineFinalScanPosX(const double travelOverhead, const SCANDIR scanDir) const
{
	const double ROIminX{ mROIcenterXY.XX - mLOIxy.XX / 2. };	//Sample edge
	const double tilePosXmin{ ROIminX + mFFOV.XX / 2. };		//The first tile center is mFFOV.XX / 2 away from the ROI edge
	const double travelX{ mLOIxy.XX - mFFOV.XX };				//The X stage does not travel the first and last mFFOV.XX / 2 from the edge of the ROI

	return determineFinalScanPos(tilePosXmin, travelX, travelOverhead, scanDir);
}

void QuickScanXY::push(const U8 *tile, const INDICES2 tileIndicesIJ)
{
	mQuickStitcher.push(tile, tileIndicesIJ);
}

void QuickScanXY::saveToFile(std::string filename, const OVERRIDE override) const
{
	mQuickStitcher.saveToFile(filename, override);
}

int QuickScanXY::tileHeight_pix() const
{
	return mQuickStitcher.tileHeight_pix();
}

int QuickScanXY::tileWidth_pix() const
{
	return mQuickStitcher.tileWidth_pix();
}
#pragma endregion "QuickScanXY"

#pragma region "Boolmap"
//Lay a tile array over the anchor pixel of the tiff
Boolmap::Boolmap(const TiffU8 &tiff, const TileArray tileArray, const PIXELS2 anchorPixel_pix, const double threshold) :
	mTiff{ tiff },
	mTileArray{ tileArray },
	mThreshold{ threshold },
	mFullHeight_pix{ tiff.heightPerFrame_pix() },
	mFullWidth_pix{ tiff.widthPerFrame_pix() },
	mNpixFull{ mFullHeight_pix * mFullWidth_pix },
	mAnchorPixel_pix{ anchorPixel_pix }
{
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");

	//Divide the large image into tiles of size tileHeight_pix * tileWidth_pix and return an array of tiles indicating if the tile is bright
	//Start scanning the tiles from the top-left corner of the image. Scan the first row from left to right. Go back and scan the second row from left to right. Etc...

	for (int II = 0; II < mTileArray.mArraySize.II; II++)
		for (int JJ = 0; JJ < mTileArray.mArraySize.JJ; JJ++)
			mIsBrightMap.push_back(isQuadrantBright_(mThreshold, { II, JJ }));
}

//Indicate if a specific tile in the array is bright. The tile indices start form 0
//II is the row index (along the image height) and JJ is the column index (along the image width) of the tile wrt the tile array
bool Boolmap::isTileBright(const INDICES2 tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.mArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row index II must be in the range [0-" + std::to_string(mTileArray.mArraySize.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.mArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The column index JJ must be in the range [0-" + std::to_string(mTileArray.mArraySize.JJ - 1) + "]");

	return mIsBrightMap.at(tileIndicesIJ.II * mTileArray.mArraySize.JJ + tileIndicesIJ.JJ);
}

//Save the boolmap as a text file
void Boolmap::saveTileMapToText(std::string filename)
{
	std::ofstream fileHandle;
	fileHandle.open(folderPath + filename + ".txt");
	for (int II = 0; II < mTileArray.mArraySize.II; II++)
	{
		for (int JJ = 0; JJ < mTileArray.mArraySize.JJ; JJ++)
			fileHandle << static_cast<int>(mIsBrightMap.at(II * mTileArray.mArraySize.JJ + JJ));
		fileHandle << "\n";	//End the row
	}
	fileHandle.close();
}

//Overlay a grid with the tiles on the tiled image
void Boolmap::saveTileGridOverlap(std::string filename, const OVERRIDE override) const
{
	const U8 lineColor{ 200 };
	const double lineThicknessFactor{ 0.7 };//If too small (<0.7), the grid will not show properly on ImageJ
	const int lineThicknessVertical{ static_cast<int>(lineThicknessFactor * mTileArray.mArraySize.JJ) };
	const int lineThicknessHorizontal{ static_cast<int>(lineThicknessFactor * mTileArray.mArraySize.II) };

	//Horizontal lines. II is the row index (along the image height) of the tile wrt the tile array
# pragma omp parallel for schedule(dynamic)
	for (int II = 0; II < mTileArray.mArraySize.II; II++)
	{
		const PIXELS2 absoluteTilePixelPos_pix{ determineTileAbsolutePixelPos_pix_({ II, 0 }) };		//Absolute tile center position wrt the Tiff		
		const int tileTopPos_pix{ absoluteTilePixelPos_pix.ii - mTileArray.mTileHeight_pix / 2 };		//Top pixels of the tile wrt the Tiff
		const int tileBottomPos_pix{ absoluteTilePixelPos_pix.ii + mTileArray.mTileHeight_pix / 2 };	//Bottom pixels of the tile wrt the Tiff
		for (int iterCol_pix = 0; iterCol_pix < mFullWidth_pix; iterCol_pix++)
			for (int iterThickness = -lineThicknessHorizontal / 2; iterThickness < lineThicknessHorizontal / 2; iterThickness++)
			{
				const int iterTopRow_pix{ (tileTopPos_pix + iterThickness) * mFullWidth_pix + iterCol_pix };
				const int iterBottomRow_pix{ (tileBottomPos_pix + iterThickness) * mFullWidth_pix + iterCol_pix };

				if (iterTopRow_pix >= 0 && iterTopRow_pix < mNpixFull)									//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterTopRow_pix] = lineColor;

				if (iterBottomRow_pix >= 0 && iterBottomRow_pix < mNpixFull)							//Make sure that the pixel is inside the Tiff
					(mTiff.data())[(tileBottomPos_pix + iterThickness) * mFullWidth_pix + iterCol_pix] = lineColor;
			}
	}

	//Vertical lines. JJ is the column index (along the image width) of the tile wrt the tile array
# pragma omp parallel for schedule(dynamic)
	for (int JJ = 0; JJ < mTileArray.mArraySize.JJ; JJ++)
	{
		const PIXELS2 absoluteTilePixelPos_pix{ determineTileAbsolutePixelPos_pix_({ 0, JJ }) };	//Absolute tile center position wrt the Tiff
		const int tileLeft{ absoluteTilePixelPos_pix.jj - mTileArray.mTileWidth_pix / 2 };			//Left pixels of the tile wrt the Tiff
		const int tileRight{ absoluteTilePixelPos_pix.jj + mTileArray.mTileWidth_pix / 2 };			//Right pixels of the tile wrt the Tiff
		for (int iterRow_pix = 0; iterRow_pix < mFullHeight_pix; iterRow_pix++)
			for (int iterThickness = -lineThicknessVertical / 2; iterThickness < lineThicknessVertical / 2; iterThickness++)
			{
				const int iterLeftColumn_pix{ iterRow_pix * mFullWidth_pix + tileLeft + iterThickness };
				const int iterRightColumn_pix{ iterRow_pix * mFullWidth_pix + tileRight + iterThickness };

				if (iterLeftColumn_pix >= 0 && iterLeftColumn_pix < mNpixFull)						//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterLeftColumn_pix] = lineColor;

				if (iterRightColumn_pix >= 0 && iterRightColumn_pix < mNpixFull)					//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterRightColumn_pix] = lineColor;
			}
	}
	mTiff.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override);
}

//Generate a Tiff with a binary dark or bright grid of tiles
void Boolmap::saveTileMap(std::string filename, const OVERRIDE override) const
{
	const U8 pixelColor{ 255 };	//Shade level
	TiffU8 tileMap{ mTiff.data(), mFullHeight_pix, mFullWidth_pix, 1 };

	//II is the row index (along the image height) and JJ is the column index (along the image width) of the tile wrt the tile array
	for (int II = 0; II < mTileArray.mArraySize.II; II++)
		for (int JJ = 0; JJ < mTileArray.mArraySize.JJ; JJ++)
		{
			if ( isTileBright({ II, JJ }) )	//If the tile is bright, shade it
			{
				const PIXELS2 absoluteTilePixelPos_pix{ determineTileAbsolutePixelPos_pix_({ II, JJ }) };	//Absolute tile center position wrt the Tiff anchor pixel
				const int tileTopPos_pix{ absoluteTilePixelPos_pix.ii - mTileArray.mTileHeight_pix / 2 };
				const int tileBottomPos_pix{ tileTopPos_pix + mTileArray.mTileHeight_pix };
				const int tileLeftPos_pix{ absoluteTilePixelPos_pix.jj - mTileArray.mTileWidth_pix / 2 };
				const int tileRightPos_pix{ tileLeftPos_pix + mTileArray.mTileWidth_pix };

				for (int iterRow_pix = tileTopPos_pix; iterRow_pix < tileBottomPos_pix; iterRow_pix++)
					for (int iterCol_pix = tileLeftPos_pix; iterCol_pix < tileRightPos_pix; iterCol_pix++)
					{
						const int iterPix{ iterRow_pix * mFullWidth_pix + iterCol_pix };
						if(iterPix >= 0 && iterPix < mNpixFull)
							(tileMap.data())[iterPix] = pixelColor;
					}
			}
		}

	tileMap.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override);
}



//Pixel position of the center of the tiles relative to the center of the Tiff
PIXELS2 Boolmap::determineTileAbsolutePixelPos_pix_(const INDICES2 tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.mArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row index II must be in the range [0-" + std::to_string(mTileArray.mArraySize.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.mArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The column index JJ must be in the range [0-" + std::to_string(mTileArray.mArraySize.JJ - 1) + "]");

	return { mAnchorPixel_pix.ii + mTileArray.determineTileRelativePixelPos_pix(tileIndicesIJ).ii,
			 mAnchorPixel_pix.jj + mTileArray.determineTileRelativePixelPos_pix(tileIndicesIJ).jj };
}

//Take the top frame of the stack and return true if it's bright. Divide the image in quadrants for a better sensitivity
//II is the row index (along the image height) and JJ is the column index (along the image width) of the tile wrt the tile array. II and JJ start from 0
bool Boolmap::isQuadrantBright_(const double threshold, const INDICES2 tileIndicesIJ) const
{
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.mArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile row index II must be in the range [0-" + std::to_string(mTileArray.mArraySize.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.mArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile column index JJ must be in the range [0-" + std::to_string(mTileArray.mArraySize.JJ - 1) + "]");

	const double nPixQuad{ 1. * mTileArray.mNpix / 4. };				//Number of pixels in a quadrant
	const int threshold_255{ static_cast<int>(threshold * 255) };		//Threshold in the range [0-255]

	//Divide the image in 4 quadrants
	const PIXELS2 absoluteTilePixelPos_pix{ determineTileAbsolutePixelPos_pix_(tileIndicesIJ) };	//Absolute tile center position wrt the Tiff anchor pixel
	const int halfHeight{ mTileArray.mTileHeight_pix / 2 };
	const int halfwidth{ mTileArray.mTileWidth_pix / 2 };
	const int tileTopPos_pix{ absoluteTilePixelPos_pix.ii - halfHeight };
	const int tileBottomPos_pix{ absoluteTilePixelPos_pix.ii + halfHeight };
	const int tileLeftPos_pix{ absoluteTilePixelPos_pix.jj - halfwidth };
	const int tileRightPos_pix{ absoluteTilePixelPos_pix.jj + halfwidth };
	std::vector<int> vec_sum;	//Vector of the sum for each quadrant

	//Make sure that the tile is inside the Tiff
	if (tileTopPos_pix >= 0 &&					//Top of the tile
		tileBottomPos_pix < mFullHeight_pix &&	//Bottom of the tile
		tileLeftPos_pix >= 0 &&					//Left of the tile
		tileRightPos_pix < mFullWidth_pix)		//Right of the tile
	{
		//Iterate over the 4 quadrants. Start scanning the quadrant from the top-left corner of the image. Scan from left to right, then go back and scan the second row from left to right.
		for (int iterQuadRow = 0; iterQuadRow < 2; iterQuadRow++)
			for (int iterQuadCol = 0; iterQuadCol < 2; iterQuadCol++)
			{
				int sum{ 0 };
				//Iterate over all the pixels inside a quadrant
				for (int iterQuadRow_pix = tileTopPos_pix + (iterQuadRow * halfHeight); iterQuadRow_pix < tileTopPos_pix + ((iterQuadRow + 1) * halfHeight); iterQuadRow_pix++)
					for (int iterQuadCol_pix = tileLeftPos_pix + (iterQuadCol * halfwidth); iterQuadCol_pix < tileLeftPos_pix + ((iterQuadCol + 1)* halfwidth); iterQuadCol_pix++)
						sum += (mTiff.data())[iterQuadRow_pix * mFullWidth_pix + iterQuadCol_pix];
				vec_sum.push_back(sum);
			}

		const double sumTL{ 1. * vec_sum.at(0) / nPixQuad };	//Average count top-left
		const double sumTR{ 1. * vec_sum.at(1) / nPixQuad };	//Average count top-right
		const double sumBL{ 1. * vec_sum.at(2) / nPixQuad };	//Average count bottom-left
		const double sumBR{ 1. * vec_sum.at(3) / nPixQuad };	//Average count bottom-right

		/*
		//For debuging
		std::cout << "Average count TL = " << sumTL << "\n";
		std::cout << "Average count TR = " << sumTR << "\n";
		std::cout << "Average count BL = " << sumBL << "\n";
		std::cout << "Average count BR = " << sumBR << "\n";
		*/
		return (sumTL > threshold_255 || sumTR > threshold_255 || sumBL > threshold_255 || sumBR > threshold_255);
		//return false;
	}
	else
		return false;
}
#pragma endregion "Boolmap"

#pragma region "Sequencer"
//Constructor using the sample's LOI. The number of stacks is calculated automatically based on the FFOV
Sequencer::Sequencer(const Sample sample, const Stack stack) :
	mSample{ sample },
	mStack{ stack },
	mTileArray{ stack.mTileHeight_pix, stack.mTileWidth_pix, determineTileArraySize_(), stack.mOverlapXYZ_frac }
{
	initializeVibratomeSlice_();
	initializeEffectiveROI_();		//Calculate the effective ROI covered by all the tiles, which might be slightly larger than the requested ROI
	reserveMemoryBlock_();			//Reserve memory for speed
}

Sequencer::Commandline Sequencer::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

//Generate a scan pattern and use the vibratome to slice the sample
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";
	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		initializeIteratorIJ_();		//Reset the tile iterator after every cut
		resetStageScanDirections_();	//Reset the scan directions of the stages after every cut

		for (std::vector<int>::size_type iterWL = 0; iterWL != mSample.mFluorLabelList.size(); iterWL++)
		{
			//The Y-stage is the slowest to react because it sits under the 2 other stages. For the best performance, iterate over II often and over JJ less often
			while (mJJ >= 0 && mJJ < mTileArray.mArraySize.JJ)		//Y-stage direction. JJ iterates from 0 to mTileArraySize.JJ
			{
				while (mII >= 0 && mII < mTileArray.mArraySize.II)	//X-stage direction. II iterates back and forth between 0 and mTileArraySize.II
				{
					moveStage_({ mII, mJJ });
					acqStack_(iterWL);
					saveStack_();
					mII -= SCANDIRtoInt(mIterScanDirXYZ.XX);	//Increase/decrease the iterator in the X-stage axis
				}
				mII += SCANDIRtoInt(mIterScanDirXYZ.XX);		//Re-initialize II by going back one step to start from 0 or mTileArraySize.II - 1
				reverseSCANDIR(mIterScanDirXYZ.XX);				//Reverse the scanning direction
				mJJ -= SCANDIRtoInt(mIterScanDirXYZ.YY);		//Increase/decrease the iterator in the Y-stage axis
			}
			mJJ += SCANDIRtoInt(mIterScanDirXYZ.YY);			//Re-initialize JJ by going back one step to start from 0 or mTileArraySize.JJ - 1
			reverseSCANDIR(mIterScanDirXYZ.YY);					//Reverse the scanning direction
		}
		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

/*
//Like generateCommandList() but instead of a list of command, Generate a list of locations to be called by frameByFrameZscanTilingXY()
//Note that this sequence does not use the vibratome
std::vector<POSITION2> Sequencer::generateLocationList()
{
	std::vector<POSITION2> locationList;
	//std::cout << "Generating the location list..." << "\n";

	int II{ 0 }, JJ{ 0 };			//Reset the tile indices after every cut
	resetStageScanDirections_();	//Reset the scan directions of the stages to the initial value

	//The stage Y is the slowest to react because it sits under the 2 other stages. For the best performance, iterate over II often and over JJ less often
	while (JJ >= 0 && JJ < mTileArraySize.JJ)			//Y-stage direction
	{
		while (II >= 0 && II < mTileArraySize.II)		//X-stage direction
		{
			const POSITION2 tileCenterXY{ tileIndicesIJToStagePosXY({ II, JJ }) };
			locationList.push_back(tileCenterXY);

			std::cout << "x = " << tileCenterXY.XX / mm << "\ty = " << tileCenterXY.YY / mm << "\n";		//For debugging
			II += SCANDIRtoInt(mIterScanDirXYZ.XX);	//Increase/decrease the iterator in the X-stage axis
		}
		II -= SCANDIRtoInt(mIterScanDirXYZ.XX);		//Re-initialize II by going back one step to start from 0 or mTileArraySize.II - 1
		reverseSCANDIR(mIterScanDirXYZ.XX);			//Reverse the scanning direction
		JJ += SCANDIRtoInt(mIterScanDirXYZ.YY);		//Increase/decrease the iterator in the Y-stage axis
	}
	JJ -= SCANDIRtoInt(mIterScanDirXYZ.YY);			//Re-initialize JJ by going back one step to start from 0 or mTileArraySize.JJ - 1
	reverseSCANDIR(mIterScanDirXYZ.YY);				//Reverse the scanning direction

	return locationList;
}
*/

int Sequencer::size() const
{
	return mCommandCounter;
}

//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
//The center the tile array is at the sample center (exact center for an odd number of tiles or slightly off center for an even number of tiles)
POSITION2 Sequencer::tileIndicesIJToStagePosXY(const INDICES2 tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.mArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + std::to_string(mTileArray.mArraySize.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.mArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + std::to_string(mTileArray.mArraySize.JJ - 1) + "]");

	//The tile centers are (1-a)*FFOV away from each other, where a*L is the tile overlap
	POSITION2 centeredIJ{ determineRelativeTileIndicesIJ(mStack.mOverlapXYZ_frac, mTileArray.mArraySize, tileIndicesIJ) };
	POSITION2 stagePosXY;
	stagePosXY.XX = mSample.mCenterXY.XX - mStack.mFFOV.XX  * centeredIJ.XX;
	stagePosXY.YY = mSample.mCenterXY.YY - mStack.mFFOV.YY  * centeredIJ.YY;

	return stagePosXY;
}

void Sequencer::printSequenceParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan direction {stageX, stageY, stageZ} = {" << SCANDIRtoInt(g_initialStageScanDirXYZ.XX) << ", " << SCANDIRtoInt(g_initialStageScanDirXYZ.YY) << ", " << SCANDIRtoInt(g_initialStageScanDirXYZ.ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Effective LOI (stageX, stageY, stageZ) = (" << effectiveLOIxyz().XX / mm << " mm, " << effectiveLOIxyz().YY / mm << " mm, " << effectiveLOIxyz().ZZ / mm << " mm)\n";
	*fileHandle << "Effective ROI boundaries [YMIN, XMIN, YMAX, XMAX] = [" << mROIeff.YMIN / mm << " mm, " << mROIeff.XMIN / mm << " mm, " << mROIeff.YMAX / mm << " mm, " << mROIeff.XMAX / mm << " mm]\n";
	*fileHandle << "Z position of the surface of the sample = " << mSample.mSurfaceZ / mm << " mm\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "Total # tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "Tile array size (stageX, stageY) = (" << mTileArray.mArraySize.II << ", " << mTileArray.mArraySize.JJ << ")\n";
	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";

	const double imagingTimePerStack{ g_lineclockHalfPeriod *  mStack.mTileHeight_pix * (mStack.mDepthZ / mStack.mPixelSizeZ) };
	const double totalImagingTime_hours{ (mStackCounter + 1) * imagingTimePerStack / seconds / 3600. };

	*fileHandle << "Runtime per stack = " << imagingTimePerStack / ms << " ms\n";
	*fileHandle << std::setprecision(6);
	*fileHandle << "Estimated total runtime (multibeam + pipelining) = " << totalImagingTime_hours << " hrs\n\n";
}

//Print the commandlist to file
void Sequencer::printToFile(const std::string fileName) const
{
	std::ofstream *fileHandle{ new std::ofstream(folderPath + fileName + ".txt") };

	*fileHandle << std::fixed;	//Show a fixed number of digits

	mSample.printParams(fileHandle);
	mSample.mFluorLabelList.printParams(fileHandle);
	printSequenceParams(fileHandle);
	mStack.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + mCommandList.front().printHeader() + "\n";
		*fileHandle << "\t\t" + mCommandList.front().printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
		//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mTileArraySize.at(X) * mTileArraySize.at(Y)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}

void Sequencer::initializeVibratomeSlice_()
{
	mScanZi = mSample.mSurfaceZ; 	//Initialize the Z-stage with the position of the surface of the sample

	mPlaneToSliceZ = mScanZi + mStack.mDepthZ - mSample.mCutAboveBottomOfStack;

	const int nZZ{ static_cast<int>(1 + std::ceil(1. / (1 - mStack.mOverlapXYZ_frac.ZZ) * (mSample.mLOIxyz_req.ZZ / mStack.mDepthZ - 1))) };
	if (nZZ > 1)
		mNtotalSlices = nZZ;		//Total number of vibratome slices in the entire sample
	else
		mNtotalSlices = 1;
}

//Calculate the number of tiles in the X-stage and Y-stage axes based on the length of interest (LOI) and the FFOV
//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
INDICES2 Sequencer::determineTileArraySize_()
{
	const int numberOfTilesII{ static_cast<int>(std::ceil(1 + 1. / (1 - mStack.mOverlapXYZ_frac.XX) * (mSample.mLOIxyz_req.XX / mStack.mFFOV.XX - 1))) };
	const int numberOfTilesJJ{ static_cast<int>(std::ceil(1 + 1. / (1 - mStack.mOverlapXYZ_frac.YY) * (mSample.mLOIxyz_req.YY / mStack.mFFOV.YY - 1))) };

	if (numberOfTilesII > 1)
		mTileArray.mArraySize.II = numberOfTilesII;		//Number of tiles in the X-stage axis
	else
		mTileArray.mArraySize.II = 1;

	if (numberOfTilesJJ > 1)
		mTileArray.mArraySize.JJ = numberOfTilesJJ;		//Number of tiles in the Y-stage axis	
	else
		mTileArray.mArraySize.JJ = 1;

	//For debugging
	//std::cout << "Number of row tiles = " << numberOfTilesII << "\tNumber of column tiles = " << numberOfTilesJJ << "\n";

	return { numberOfTilesII, numberOfTilesJJ };
}

//Calculate the effective ROI =  { YMIN, XMIN, YMAX, XMAX } covered by all the tiles
void Sequencer::initializeEffectiveROI_()
{
	//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
	const POSITION2 tilePosXYmin = tileIndicesIJToStagePosXY({ 0, 0 });															//Absolute position of the CENTER of the tile
	const POSITION2 tilePosXYmax = tileIndicesIJToStagePosXY({ mTileArray.mArraySize.II - 1, mTileArray.mArraySize.JJ - 1 });	//Absolute position of the CENTER of the tile

	//The ROI is measured from the border of the tiles. Therefore, add half of the FFOV
	mROIeff.XMAX = tilePosXYmin.XX + mStack.mFFOV.XX / 2.;
	mROIeff.YMAX = tilePosXYmin.YY + mStack.mFFOV.YY / 2.;
	mROIeff.XMIN = tilePosXYmax.XX - mStack.mFFOV.XX / 2.;
	mROIeff.YMIN = tilePosXYmax.YY - mStack.mFFOV.YY / 2.;
}

//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
void Sequencer::reserveMemoryBlock_()
{
	const int nTotalTilesPerVibratomeSlice{ mTileArray.mArraySize.II * mTileArray.mArraySize.JJ };											//Total number of tiles in a vibratome slice
	const int nTotalTilesEntireSample{ mNtotalSlices * static_cast<int>(mSample.mFluorLabelList.size()) * nTotalTilesPerVibratomeSlice };	//Total number of tiles in the entire sample. mNtotalSlices is fixed at 1
	mCommandList.reserve(3 * nTotalTilesEntireSample + mNtotalSlices - 1);
}

//Reset the iterators mII and mJJ to the initial values
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::initializeIteratorIJ_()
{
	switch (mIterScanDirXYZ.XX)
	{
	case SCANDIR::RIGHTWARD:	//The X-stage moves to the right, therefore, the sample is imaged from right to left
		mII = mTileArray.mArraySize.II - 1;
		break;
	case SCANDIR::LEFTWARD:		//The X-stage moves to the left, therefore, the sample is imaged from left to right
		mII = 0;
		break;
	}

	switch (mIterScanDirXYZ.YY)
	{
	case SCANDIR::INWARD:		//The Y-stage moves inward, therefore, the sample is imaged from "inside" to "outside" of the microscope
		mJJ = mTileArray.mArraySize.JJ - 1;
		break;
	case SCANDIR::OUTWARD:		//The Y-stage moves outward, therefore, the sample is imaged from "outside" to "inside" of the microscope
		mJJ = 0;
		break;
	}
}

void Sequencer::resetStageScanDirections_()
{
	mIterScanDirXYZ = g_initialStageScanDirXYZ;
}

//Convert a ROI = {ymin, xmin, ymax, xmax} to the equivalent length of interest (LOI) in the X-stage and Y-stage axes
SIZE3 Sequencer::effectiveLOIxyz() const
{
	return { mROIeff.XMAX - mROIeff.XMIN,
			 mROIeff.YMAX - mROIeff.YMIN,
			 mStack.mDepthZ * ((1 - mStack.mOverlapXYZ_frac.ZZ) * (mNtotalSlices - 1) + 1) };
}

//Move the stage to the position corresponding to the tile indices II and JJ 
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::moveStage_(const INDICES2 tileIJ)
{
	if (tileIJ.II < 0 || tileIJ.II >= mTileArray.mArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + std::to_string(mTileArray.mArraySize.II - 1) + "]");
	if (tileIJ.JJ < 0 || tileIJ.JJ >= mTileArray.mArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + std::to_string(mTileArray.mArraySize.JJ - 1) + "]");

	const POSITION2 tileCenterXY = tileIndicesIJToStagePosXY(tileIJ);

	Commandline commandline{ Action::ID::MOV };
	commandline.mParam.moveStage = { mSliceCounter, tileIJ, tileCenterXY };
	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequencer::acqStack_(const int wavelengthIndex)
{
	if (wavelengthIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength index must be >= 0");

	//Read the corresponding laser configuration
	const FluorLabelList::FluorLabel fluorLabel{ mSample.mFluorLabelList.at(wavelengthIndex) };

	Commandline commandline{ Action::ID::ACQ };
	commandline.mParam.acqStack = { mStackCounter, fluorLabel.mWavelength_nm, mIterScanDirXYZ.ZZ, mScanZi, mStack.mDepthZ, fluorLabel.mScanPmin, fluorLabel.mScanPexp, fluorLabel.nFramesBinning };
	mCommandList.push_back(commandline);
	mStackCounter++;	//Count the number of stacks acquired
	mCommandCounter++;	//Count the number of commands

	//Update the parameters for the next iteration
	reverseSCANDIR(mIterScanDirXYZ.ZZ);								//Reverse the scanning direction in the Z-stage axis
}

void Sequencer::saveStack_()
{
	Commandline commandline{ Action::ID::SAV };
	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

void Sequencer::cutSlice_()
{
	//Move the sample to face the vibratome blade. Note the additional offset in the Z-stage axis
	const POSITION3 samplePosXYZ{ mSample.mBladePosXY.XX, mSample.mBladePosXY.YY, mPlaneToSliceZ + mSample.mBladeFocalplaneOffsetZ };

	Commandline commandline{ Action::ID::CUT };
	commandline.mParam.cutSlice = { samplePosXYZ };
	mCommandList.push_back(commandline);
	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the Z-stage and the height of the plane to be cut in the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	mScanZi += (1 - mStack.mOverlapXYZ_frac.ZZ) * mStack.mDepthZ;
	mPlaneToSliceZ += (1 - mStack.mOverlapXYZ_frac.ZZ) * mStack.mDepthZ;
}
#pragma endregion "sequencer"