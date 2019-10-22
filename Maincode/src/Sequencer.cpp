#include "Sequencer.h"

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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

//Take the integer indices II, JJ = 0, 1, 2... of the array of tiles and return new tile indices (now of double type) with overlapping tiles.
//The returned indices are wrt the center tile and can be negative
//For an odd number of tiles, the center tile is at the middle of the tile array
//For an even number of tiles, there are 2 tiles in the middle of the tile array. The one on the top/left is taken as the reference tile
POSITION2 determineRelativeTileIndicesIJ(const TILEOVERLAP3 overlapIJK_frac, const TILEDIM2 tileArraySizeIJ, const TILEIJ tileIndicesIJ)
{
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.KK < 0 || overlapIJK_frac.II > 1 || overlapIJK_frac.JJ > 1 || overlapIJK_frac.KK > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-1]");
	if (tileArraySizeIJ.II <= 0 || tileArraySizeIJ.II <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile size must be > 0");
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= tileArraySizeIJ.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + Util::toString(tileArraySizeIJ.II, 0) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= tileArraySizeIJ.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + Util::toString(tileArraySizeIJ.JJ, 0) + "]");

	return { (1. - overlapIJK_frac.II) * (tileIndicesIJ.II - static_cast<int>((tileArraySizeIJ.II - 1) / 2)),
			 (1. - overlapIJK_frac.JJ) * (tileIndicesIJ.JJ - static_cast<int>((tileArraySizeIJ.JJ - 1) / 2)) };
}

double determineInitialScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel range must be > 0");
	if (travelOverhead < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel overhead must be >= 0");

	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin - travelOverhead;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin + travel + travelOverhead;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

//Travel overhead to avoid the nonlinearity at the end of the stage scan
double determineFinalScanPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel range must be > 0");
	if (travelOverhead < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel overhead must be >= 0");

	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin + travel + travelOverhead;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin - travelOverhead;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

//The initial laser power for scanning a stack depends on whether the stack is imaged from the top down or from the bottom up
//totalPowerInc is the total increase and NOT the increase per unit of length
double determineInitialLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power must be >= 0");
	if (totalPowerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power increase must be >= 0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin;
	case SCANDIR::DOWNWARD:
		return powerMin + totalPowerInc;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

//totalPowerInc is the total increase and NOT the increase per unit of length
double determineFinalLaserPower(const double powerMin, const double totalPowerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power must be >= 0");
	if (totalPowerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power increase must be >= 0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin + totalPowerInc;
	case SCANDIR::DOWNWARD:
		return powerMin;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

#pragma region "TileArray"
TileArray::TileArray(const int tileHeight_pix, const int tileWidth_pix, const TILEDIM2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac) :
	mTileHeight_pix{ tileHeight_pix },
	mTileWidth_pix{ tileWidth_pix },
	mArraySizeIJ{ tileArraySizeIJ },
	mOverlapIJK_frac{ overlapIJK_frac }
{
	if (tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile height must be > 0");
	if (tileWidth_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile width must be > 0");
	if (tileArraySizeIJ.II <= 0 || tileArraySizeIJ.II <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile size must be > 0");
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.KK < 0 || overlapIJK_frac.II > 1 || overlapIJK_frac.JJ > 1 || overlapIJK_frac.KK > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-1]");
}

TileArray::TileArray(const PIXDIM2 tileSize_pix, const TILEDIM2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac):
	TileArray(tileSize_pix.ii, tileSize_pix.jj, tileArraySizeIJ, overlapIJK_frac)
{}

int TileArray::readTileHeight_pix() const
{
	return mTileHeight_pix;
}

int TileArray::readTileWidth_pix() const
{
	return mTileWidth_pix;
}

TILEDIM2 TileArray::readTileArraySizeIJ() const
{
	return mArraySizeIJ;
}

int TileArray::readTileArraySizeIJ(const TileArray::Axis axis) const
{
	switch (axis)
	{
	case TileArray::Axis::II:
		return mArraySizeIJ.II;
	case TileArray::Axis::JJ:
		return mArraySizeIJ.JJ;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected tile array axis unavailable");
	}
}

TILEOVERLAP3 TileArray::readTileOverlapIJK_frac() const
{
	return mOverlapIJK_frac;
}

//Pixel position of the center of the tiles relative to the center of the array
PIXELij TileArray::determineTileRelativePixelPos_pix(const TILEIJ tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mArraySizeIJ.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row index II must be in the range [0-" + std::to_string(mArraySizeIJ.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mArraySizeIJ.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The column index JJ must be in the range [0-" + std::to_string(mArraySizeIJ.JJ - 1) + "]");

	POSITION2 relativeTileIndicesIJ{ determineRelativeTileIndicesIJ(mOverlapIJK_frac, mArraySizeIJ, tileIndicesIJ) };

	return { static_cast<int>(std::round(mTileHeight_pix * relativeTileIndicesIJ.XX)),
			 static_cast<int>(std::round(mTileWidth_pix * relativeTileIndicesIJ.YY)) };
}
#pragma endregion "TileArray"

#pragma region "QuickStitcher"
//tileHeight_pix = tile height, tileWidth_pix = tile width, tileArraySizeIJ = { number of tiles as rows, number of tiles as columns}
//II is the row index (along the image height) and JJ is the column index (along the image width) wrt the tile array. II and JJ start from 0
QuickStitcher::QuickStitcher(const int tileHeight_pix, const int tileWidth_pix, const TILEDIM2 tileArraySizeIJ, const TILEOVERLAP3 overlapIJK_frac) :
	TiffU8{ tileHeight_pix * tileArraySizeIJ.II, tileWidth_pix * tileArraySizeIJ.JJ, 1 },
	TileArray{ tileHeight_pix,
				tileWidth_pix,
				tileArraySizeIJ,
				overlapIJK_frac },
	mFullHeight{ tileHeight_pix * tileArraySizeIJ.II},
	mFullWidth{ tileWidth_pix * tileArraySizeIJ.JJ }
{
	if (tileHeight_pix <= 0 || tileWidth_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile height and width must be > 0");
	if (tileArraySizeIJ.II <= 0 || tileArraySizeIJ.JJ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The array dimensions must be > 0");
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.KK < 0 || overlapIJK_frac.II > 1 || overlapIJK_frac.JJ > 1 || overlapIJK_frac.KK > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-1]");
}

//II is the row index (along the image height) and JJ is the column index (along the image width) wrt the tile array. II and JJ start from 0
void QuickStitcher::push(const U8 *tile, const TILEIJ tileIndicesIJ)
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= TileArray::readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile row index II must be in the range [0-" + std::to_string(TileArray::readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= TileArray::readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile column index JJ must be in the range [0-" + std::to_string(TileArray::readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	//const int tileHeight_pix{ tile.heightPerFrame_pix() };													//Height of the tile
	//const int tileWidth_pix{ tile.widthPerFrame_pix() };														//Width of the tile
	const int rowShift_pix{ tileIndicesIJ.II *  TileArray::readTileHeight_pix() };								//In the mTiff image, shift the tile down by this many pixels
	const int colShift_pix{ tileIndicesIJ.JJ *  TileArray::readTileWidth_pix() };								//In the mTiff image, shift the tile to the right by this many pixels
	const int tileBytesPerRow{ TileArray::readTileWidth_pix() * static_cast<int>(sizeof(U8)) };					//Bytes per row of the input tile
	const int stitchedTiffBytesPerRow{ TiffU8::readWidthPerFrame_pix() * static_cast<int>(sizeof(U8)) };		//Bytes per row of the tiled image

	/*
	//Transfer the data from the input tile to mStitchedTiff. Old way: copy pixel by pixel
	for (int iterCol_pix = 0; iterCol_pix < tileWidth_pix; iterCol_pix++)
		for (int iterRow_pix = 0; iterRow_pix < tileHeight_pix; iterRow_pix++)
			mStitchedTiff.data()[(rowShift_pix + iterRow_pix) * mStitchedTiff.widthPerFrame_pix() + colShift_pix + iterCol_pix] = tile.data()[iterRow_pix * tileWidth_pix + iterCol_pix];*/

			//Transfer the data from the input tile to mStitchedTiff. New way: copy row by row
	for (int iterRow_pix = 0; iterRow_pix < TileArray::readTileHeight_pix(); iterRow_pix++)
		std::memcpy(&TiffU8::data()[(rowShift_pix + iterRow_pix) * stitchedTiffBytesPerRow + colShift_pix * sizeof(U8)], &tile[iterRow_pix * tileBytesPerRow], tileBytesPerRow);
}

void QuickStitcher::saveToFile(std::string filename, const OVERRIDE override) const
{
	TiffU8::saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override, SCANDIR::UPWARD);
}

int QuickStitcher::readFullHeight_pix() const
{
	return mFullHeight;
}

int QuickStitcher::readFullWidth_pix() const
{
	return mFullWidth;
}
#pragma endregion "QuickStitcher"

#pragma region "PanoramicScan"
PanoramicScan::PanoramicScan(const POSITION2 ROIcenterXY, const FFOV2 FFOV, const LENGTH2 pixelSizeXY, const LENGTH2 LOIxy) :
	mROIcenterXY{ ROIcenterXY },
	mFFOV{ FFOV },
	mPixelSizeXY{ pixelSizeXY },													//Pixel resolution (unit of length per pixel)
	mLOIxy{ castLOIxy_(FFOV, LOIxy) },												//Cast the length of interest (LOI) to odd numbers to have an exact center tile. LOI covers from tile edge to tile edge
	mPanoramicWidth_pix{ Util::intceil(castLOIxy_(FFOV, LOIxy).YY / pixelSizeXY.YY) },	//Full width. Note that the cast mLOIxy is being used
	QuickStitcher{ Util::intceil(castLOIxy_(FFOV, LOIxy).XX / pixelSizeXY.XX),		//tileHeight_pix. The tile height is the same as the full height (because a tile is a long vertical strip). Note that the cast mLOIxy is being used
				   Util::intceil(FFOV.YY / pixelSizeXY.YY),							//tileWidth_pix
				   { 1, Util::intceil(castLOIxy_(FFOV, LOIxy).YY / FFOV.YY) },		//tile array size = { 1, JJ }. Only 1 row and many columns. Note that the cast mLOIxy is being used
				   { 0, 0, 0} }														//No overlap	
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FFOV must be > 0");
	if (pixelSizeXY.XX <= 0 || pixelSizeXY.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel size must be > 0");
	if (LOIxy.XX <= 0 || LOIxy.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The length of interest must be > 0");

	for (int JJ = 0; JJ < QuickStitcher::readTileArraySizeIJ(TileArray::Axis::JJ); JJ++)
	{
		const int II{ 0 };	//Only 1 row
		const POSITION2 relativeTileIndexIJ{ determineRelativeTileIndicesIJ(readTileOverlapIJK_frac(), readTileArraySizeIJ(), { II, JJ }) };
		mStageYpos.push_back(mROIcenterXY.YY - mFFOV.YY * relativeTileIndexIJ.YY);						//for now, only stitching strips to the right (i.e. move the stage to the left)
	}
}

double PanoramicScan::determineInitialScanPosX(const double travelOverhead, const SCANDIR scanDir) const
{
	if (travelOverhead < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel overhead must be > 0");

	const double ROIminX{ mROIcenterXY.XX - mLOIxy.XX / 2 };	//Sample edge
	const double tilePosXmin{ ROIminX + mFFOV.XX / 2. };		//LOI covers from tile edge to tile edge. The first tile center is mFFOV.XX / 2 away from the ROI edge
	const double travelX{ mLOIxy.XX - mFFOV.XX };				//The X stage does not travel the first and last mFFOV.XX / 2 from the edge of the ROI

	return determineInitialScanPos(tilePosXmin, travelX, travelOverhead, scanDir);
}

double PanoramicScan::determineFinalScanPosX(const double travelOverhead, const SCANDIR scanDir) const
{
	if (travelOverhead < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The travel overhead must be > 0");

	const double ROIminX{ mROIcenterXY.XX - mLOIxy.XX / 2. };	//Sample edge
	const double tilePosXmin{ ROIminX + mFFOV.XX / 2. };		//LOI covers from tile edge to tile edge. The first tile center is mFFOV.XX / 2 away from the ROI edge
	const double travelX{ mLOIxy.XX - mFFOV.XX };				//The X stage does not travel the first and last mFFOV.XX / 2 from the edge of the ROI

	return determineFinalScanPos(tilePosXmin, travelX, travelOverhead, scanDir);
}

int PanoramicScan::readNumberStageYpos() const
{
	return static_cast<int>(mStageYpos.size());
}

double PanoramicScan::readStageYposFront() const
{
	return mStageYpos.front();
}

double PanoramicScan::readStageYposBack() const
{
	return mStageYpos.back();
}

double PanoramicScan::readStageYposAt(const int index) const
{
	return mStageYpos.at(index);
}

//Cast the input to int and return an odd int
int PanoramicScan::castToOddnumber_(const double inputNumber) const
{
	int inputNumber_int{ Util::intceil(inputNumber) };
	if (inputNumber_int % 2)	//Odd number
		return inputNumber_int;
	else				//Even number
	{
		//std::cerr << "WARNING in " << __FUNCTION__ << ": The number of tiles has been cast to an odd number\n";
		return inputNumber_int + 1;
	}
}

//Return a cast LOI that covers an odd number of tiles
LENGTH2 PanoramicScan::castLOIxy_(const FFOV2 FFOV, const LENGTH2 LOIxy) const
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FFOV must be > 0");
	if (LOIxy.XX <= 0 || LOIxy.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The length of interest must be > 0");

	const int numberOfTilesII{ castToOddnumber_(LOIxy.XX / FFOV.XX) };
	const int numberOfTilesJJ{ castToOddnumber_(LOIxy.YY / FFOV.YY) };

	return { numberOfTilesII * FFOV.XX , numberOfTilesJJ * FFOV.YY };
}
#pragma endregion "PanoramicScan"

#pragma region "Boolmap"
//Lay a tile array over the center of the tiff. LOI is the length of interest
Boolmap::Boolmap(const TiffU8 &tiff, const TILEDIM2 tileArraySizeIJ, const PIXDIM2 tileSizeij_pix, const TILEOVERLAP3 overlapIJK_frac, const double threshold) :
	mTiff{ tiff },
	mTileArray{ tileSizeij_pix ,
				{ tileArraySizeIJ.II, tileArraySizeIJ.JJ },
				overlapIJK_frac },
	mThreshold{ threshold },
	mPanoramicHeight_pix{ tiff.readHeightPerFrame_pix() },
	mPanoramicWidth_pix{ tiff.readWidthPerFrame_pix() },
	mNpixPanoramic{ tiff.readHeightPerFrame_pix() * tiff.readWidthPerFrame_pix() },
	mAnchorPixel_pix{ tiff.readHeightPerFrame_pix() / 2,
					  tiff.readWidthPerFrame_pix() / 2}		//Set the anchor pixels to the center of the image
{
	if (tileSizeij_pix.ii <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile height must be > 0");
	if (tileSizeij_pix.jj <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile width must be > 0");
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.KK < 0 || overlapIJK_frac.II > 1 || overlapIJK_frac.JJ > 1 || overlapIJK_frac.KK > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-1]");
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");

	generateBoolmap_();
}

Boolmap::Boolmap(const PanoramicScan &panoramicScan, const TILEDIM2 tileArraySizeIJ, const PIXDIM2 tileSizeij_pix, const TILEOVERLAP3 overlapIJK_frac, const double threshold):
	mTiff{ panoramicScan.data(),
		   panoramicScan.readFullHeight_pix(),
		   panoramicScan.readFullWidth_pix(),
		   1 },
	mTileArray{ tileSizeij_pix,
				{ tileArraySizeIJ.II, tileArraySizeIJ.JJ },
				overlapIJK_frac },
	mThreshold{ threshold },
	mPanoramicHeight_pix{ panoramicScan.readFullHeight_pix() },
	mPanoramicWidth_pix{ panoramicScan.readFullWidth_pix() },
	mNpixPanoramic{ panoramicScan.readFullHeight_pix() * panoramicScan.readFullWidth_pix() },
	mAnchorPixel_pix{ panoramicScan.readFullHeight_pix() / 2,
					  panoramicScan.readFullWidth_pix() / 2 }		//Set the anchor pixels to the center of the image
{
	if (tileSizeij_pix.ii <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile height must be > 0");
	if (tileSizeij_pix.jj <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile width must be > 0");
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.KK < 0 || overlapIJK_frac.II > 1 || overlapIJK_frac.JJ > 1 || overlapIJK_frac.KK > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap must be in the range [0-1]");
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");

	generateBoolmap_();
}

//Indicate if a specific tile in the array is bright. The tile indices start form 0
//II is the row index (along the image height) and JJ is the column index (along the image width) wrt the tile array
bool Boolmap::isTileBright(const TILEIJ tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row index II must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The column index JJ must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	try
	{
		return mBoolmap.at(tileIndicesIJ.II * mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) + tileIndicesIJ.JJ);
	}
	catch (...) {
		throw std::out_of_range((std::string)__FUNCTION__);
	}	
}

//Save the boolmap as a text file
void Boolmap::saveBoolmapToText(std::string filename, const OVERRIDE override)
{
	std::ofstream fileHandle;

	if (override == OVERRIDE::DIS)
		filename = Util::doesFileExist(filename, ".txt");

	fileHandle.open(g_folderPath + filename + ".txt");

	try
	{
		for (int II = 0; II < mTileArray.readTileArraySizeIJ(TileArray::Axis::II); II++)
		{
			for (int JJ = 0; JJ < mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ); JJ++)
				fileHandle << static_cast<int>(mBoolmap.at(II * mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) + JJ));
			fileHandle << "\n";	//End the row
		}
	}
	catch (...) {
		throw std::out_of_range((std::string)__FUNCTION__);
	}

	fileHandle.close();
}

//Overlay a grid with the tiles on the stitched image
void Boolmap::saveTileGridOverlay(std::string filename, const OVERRIDE override) const
{
	const U8 lineColor{ 200 };
	const int lineThickness{ 6 };
	//const double lineThicknessFactor{ 0.1 };//If too small (<0.7), the grid will not show properly on ImageJ
	//const int lineThicknessVertical{ static_cast<int>(lineThicknessFactor * mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ)) };
	//const int lineThicknessHorizontal{ static_cast<int>(lineThicknessFactor * mTileArray.readTileArraySizeIJ(TileArray::Axis::II)) };

	//Horizontal lines. II is the row index (along the image height) wrt the tile array
# pragma omp parallel for schedule(dynamic)
	for (int II = 0; II < mTileArray.readTileArraySizeIJ(TileArray::Axis::II); II++)
	{
		const PIXELij tileCenterPos_pix{ determineTileAbsolutePixelPos_pix_({ II, 0 }) };			//Tile center position wrt the Tiff		
		const int tileTopPos_pix{ tileCenterPos_pix.ii - mTileArray.readTileHeight_pix() / 2 };		//Top pixels of the tile wrt the Tiff
		const int tileBottomPos_pix{ tileCenterPos_pix.ii + mTileArray.readTileHeight_pix() / 2 };	//Bottom pixels of the tile wrt the Tiff
		for (int iterCol_pix = 0; iterCol_pix < mPanoramicWidth_pix; iterCol_pix++)
			for (int iterThickness = -lineThickness / 2; iterThickness < lineThickness / 2; iterThickness++)
			{
				const int iterTopRow_pix{ (tileTopPos_pix + iterThickness) * mPanoramicWidth_pix + iterCol_pix };
				const int iterBottomRow_pix{ (tileBottomPos_pix + iterThickness) * mPanoramicWidth_pix + iterCol_pix };

				if (iterTopRow_pix >= 0 && iterTopRow_pix < mNpixPanoramic)								//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterTopRow_pix] = lineColor;

				if (iterBottomRow_pix >= 0 && iterBottomRow_pix < mNpixPanoramic)						//Make sure that the pixel is inside the Tiff
					(mTiff.data())[(tileBottomPos_pix + iterThickness) * mPanoramicWidth_pix + iterCol_pix] = lineColor;
			}
	}

	//Vertical lines. JJ is the column index (along the image width) wrt the tile array
# pragma omp parallel for schedule(dynamic)
	for (int JJ = 0; JJ < mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ); JJ++)
	{
		const PIXELij tileCenterPos_pix{ determineTileAbsolutePixelPos_pix_({ 0, JJ }) };			//Tile center position wrt the Tiff
		const int tileLeft{ tileCenterPos_pix.jj - mTileArray.readTileWidth_pix() / 2 };			//Left pixels of the tile wrt the Tiff
		const int tileRight{ tileCenterPos_pix.jj + mTileArray.readTileWidth_pix() / 2 };			//Right pixels of the tile wrt the Tiff
		for (int iterRow_pix = 0; iterRow_pix < mPanoramicHeight_pix; iterRow_pix++)
			for (int iterThickness = -lineThickness / 2; iterThickness < lineThickness / 2; iterThickness++)
			{
				const int iterLeftColumn_pix{ iterRow_pix * mPanoramicWidth_pix + tileLeft + iterThickness };
				const int iterRightColumn_pix{ iterRow_pix * mPanoramicWidth_pix + tileRight + iterThickness };

				if (iterLeftColumn_pix >= 0 && iterLeftColumn_pix < mNpixPanoramic)						//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterLeftColumn_pix] = lineColor;

				if (iterRightColumn_pix >= 0 && iterRightColumn_pix < mNpixPanoramic)					//Make sure that the pixel is inside the Tiff
					(mTiff.data())[iterRightColumn_pix] = lineColor;
			}
	}
	mTiff.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override);
}

//Save a copy of the input Tiff with the dark tiles shaded
void Boolmap::saveTileMap(std::string filename, const OVERRIDE override) const
{
	TiffU8 outputTiff{ mPanoramicHeight_pix, mPanoramicWidth_pix, 1 };	//Create an empty Tiff

	//Shade the Tiff with a chosen grey level
	const U8 pixelColor{ 200 };	
	for (int iterPixel = 0; iterPixel < mNpixPanoramic; iterPixel++)
		(outputTiff.data())[iterPixel] = pixelColor;

	//II is the row index (along the image height) and JJ is the column index (along the image width) wrt the tile array
	for (int II = 0; II < mTileArray.readTileArraySizeIJ(TileArray::Axis::II); II++)
		for (int JJ = 0; JJ < mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ); JJ++)
			if (isTileBright({ II, JJ }))	//If the tile is bright, copy the pixel from mTiff.data()
			{
				const PIXELij tileCenterPos_pix{ determineTileAbsolutePixelPos_pix_({ II, JJ }) };	//Tile center position wrt the Tiff anchor pixel
				const int tileTopPos_pix{ tileCenterPos_pix.ii - mTileArray.readTileHeight_pix() / 2 };
				const int tileBottomPos_pix{ tileTopPos_pix + mTileArray.readTileHeight_pix() };
				const int tileLeftPos_pix{ tileCenterPos_pix.jj - mTileArray.readTileWidth_pix() / 2 };
				const int tileRightPos_pix{ tileLeftPos_pix + mTileArray.readTileWidth_pix() };

				for (int iterRow_pix = tileTopPos_pix; iterRow_pix < tileBottomPos_pix; iterRow_pix++)
					for (int iterCol_pix = tileLeftPos_pix; iterCol_pix < tileRightPos_pix; iterCol_pix++)
						if (iterRow_pix >= 0 && iterRow_pix < mPanoramicHeight_pix && iterCol_pix >= 0 && iterCol_pix < mPanoramicWidth_pix)//Check that the tile is inside the Tiff
							(outputTiff.data())[iterRow_pix * mPanoramicWidth_pix + iterCol_pix] = (mTiff.data())[iterRow_pix * mPanoramicWidth_pix + iterCol_pix];
			}
	outputTiff.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override);
}

//Pixel position of the center of the tiles relative to the center of the Tiff
PIXELij Boolmap::determineTileAbsolutePixelPos_pix_(const TILEIJ tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row index II must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The column index JJ must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	return { mAnchorPixel_pix.ii + mTileArray.determineTileRelativePixelPos_pix(tileIndicesIJ).ii,
			 mAnchorPixel_pix.jj + mTileArray.determineTileRelativePixelPos_pix(tileIndicesIJ).jj };
}

//Take the top frame of the stack and return true if it's bright. Divide the image in quadrants for a better sensitivity
//II is the row index (along the image height) and JJ is the column index (along the image width) wrt the tile array. II and JJ start from 0
bool Boolmap::isQuadrantBright_(const double threshold, const TILEIJ tileIndicesIJ) const
{
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile row index II must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile column index JJ must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	const int threshold_255{ static_cast<int>(threshold * 255) };		//Threshold in the range [0-255]

	//Divide the image in 4 quadrants
	const PIXELij tileCenterPos_pix{ determineTileAbsolutePixelPos_pix_(tileIndicesIJ) };	//Tile center position wrt the Tiff anchor pixel
	const int halfHeight{ mTileArray.readTileHeight_pix() / 2 };
	const int halfwidth{ mTileArray.readTileWidth_pix() / 2 };
	const int tileTopPos_pix{ tileCenterPos_pix.ii - halfHeight };
	const int tileBottomPos_pix{ tileCenterPos_pix.ii + halfHeight };
	const int tileLeftPos_pix{ tileCenterPos_pix.jj - halfwidth };
	const int tileRightPos_pix{ tileCenterPos_pix.jj + halfwidth };
	std::vector<double> vec_sum;	//Vector of the sum for each quadrant

	//Iterate over the 4 quadrants. Start scanning the quadrant from the top-left corner of the image. Scan from left to right, then go back and scan the second row from left to right.
	for (int iterQuadRow = 0; iterQuadRow < 2; iterQuadRow++)
		for (int iterQuadCol = 0; iterQuadCol < 2; iterQuadCol++)
		{
			int sum{ 0 };
			int nPixQuad{ 0 };		//Calculate the number of pixels in a quadrant dynamically to take into account the fractional tiles at the tiff edges

			//Iterate over all the pixels inside a quadrant
			for (int iterQuadRow_pix = tileTopPos_pix + (iterQuadRow * halfHeight); iterQuadRow_pix < tileTopPos_pix + ((iterQuadRow + 1) * halfHeight); iterQuadRow_pix++)
				for (int iterQuadCol_pix = tileLeftPos_pix + (iterQuadCol * halfwidth); iterQuadCol_pix < tileLeftPos_pix + ((iterQuadCol + 1)* halfwidth); iterQuadCol_pix++)
				{
					const int pixelIndex{ iterQuadRow_pix * mPanoramicWidth_pix + iterQuadCol_pix };
					if (pixelIndex >= 0 && pixelIndex < mPanoramicHeight_pix * mPanoramicWidth_pix)
					{
						sum += (mTiff.data())[pixelIndex];
						nPixQuad++;
					}
				}
			vec_sum.push_back(1. * sum / nPixQuad);
		}

	const double sumTL{ 1. * vec_sum.at(0) };	//Average count top-left
	const double sumTR{ 1. * vec_sum.at(1) };	//Average count top-right
	const double sumBL{ 1. * vec_sum.at(2) };	//Average count bottom-left
	const double sumBR{ 1. * vec_sum.at(3) };	//Average count bottom-right

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

//Note that when the tiles have some overlap, the tile array will not cover the entire Tiff
void Boolmap::generateBoolmap_()
{
	//Divide the large image into tiles of size tileHeight_pix * tileWidth_pix and return an array of tiles indicating if the tile is bright
	//Start scanning the tiles from the top-left corner of the image. Scan the first row from left to right. Go back and scan the second row from left to right. Etc...
	for (int II = 0; II < mTileArray.readTileArraySizeIJ(TileArray::Axis::II); II++)
		for (int JJ = 0; JJ < mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ); JJ++)
			mBoolmap.push_back(isQuadrantBright_(mThreshold, { II, JJ }));
}

void Boolmap::fillTileMapHoles()
{
	const TILEDIM2 tileArraySizeIJ{ mTileArray.readTileArraySizeIJ() };

	std::vector<int> minII(tileArraySizeIJ.JJ, tileArraySizeIJ.II-1);	//min II for each JJ. Vector of length tileArraySizeIJ.JJ initialized with the max II = tileArraySizeIJ.II - 1
	std::vector<int> maxII(tileArraySizeIJ.JJ, 0);						//max II for each JJ. Vector of length tileArraySizeIJ.JJ initialized with the min II = 0
	std::vector<int> minJJ(tileArraySizeIJ.II, tileArraySizeIJ.JJ-1);	//min JJ for each II. Vector of length tileArraySizeIJ.II initialized with the max JJ = tileArraySizeIJ.JJ - 1
	std::vector<int> maxJJ(tileArraySizeIJ.II, 0);						//max JJ for each II. Vector of length tileArraySizeIJ.II initialized with the min JJ = 0

	//Find the min and max IIs and JJs
	for (int II = 0; II < tileArraySizeIJ.II; II++)
		for (int JJ = 0; JJ < tileArraySizeIJ.JJ; JJ++)
			if (mBoolmap.at(II * tileArraySizeIJ.JJ + JJ) == 1)
			{
				if (II < minII.at(JJ))
					minII.at(JJ) = II;
				if (II > maxII.at(JJ))
					maxII.at(JJ) = II;
				if (JJ < minJJ.at(II))
					minJJ.at(II) = JJ;
				if (JJ > maxJJ.at(II))
					maxJJ.at(II) = JJ;
			}

	//For debugging
	//for (int II = 0; II < mTileArray.readTileArraySizeIJ(TileArray::Axis::II); II++)
	//	std::cout << minJJ.at(II) << "\n";

	//Fill the holes in mBoolmap
	for (int II = 0; II < tileArraySizeIJ.II; II++)
		for (int JJ = 0; JJ < tileArraySizeIJ.JJ; JJ++)
			if (II >= minII.at(JJ) && II <= maxII.at(JJ) && JJ >= minJJ.at(II) && JJ <= maxJJ.at(II))
				mBoolmap.at(II * tileArraySizeIJ.JJ + JJ) = 1;

	//Enlarge the boolmap by 1 place in the vertical direction of the tiff
	std::vector<bool> mBoolmapAugmentedV{ mBoolmap };
	for (int JJ = 0; JJ < tileArraySizeIJ.JJ; JJ++)
	{
		//
		if (mBoolmapAugmentedV.at(minII.at(JJ) * tileArraySizeIJ.JJ + JJ) == 1)
		{
			const int minIIminus1{ (minII.at(JJ) - 1) * tileArraySizeIJ.JJ + JJ };
			if (minIIminus1 >= 0)
				mBoolmapAugmentedV.at(minIIminus1) = 1;
		}	

		if (mBoolmapAugmentedV.at(maxII.at(JJ) * tileArraySizeIJ.JJ + JJ) == 1)
		{
			const int maxIIplus1{ (maxII.at(JJ) + 1) * tileArraySizeIJ.JJ + JJ };
			if (maxIIplus1 < tileArraySizeIJ.II * tileArraySizeIJ.JJ)
				mBoolmapAugmentedV.at(maxIIplus1) = 1;
		}
	}

	//Enlarge the boolmap by 1 place in the horizontal direction of the tiff
	std::vector<bool> mBoolmapAugmentedH{ mBoolmap };
	for (int II = 0; II < tileArraySizeIJ.II; II++)
	{
		if (mBoolmapAugmentedH.at(II * tileArraySizeIJ.JJ + minJJ.at(II)) == 1)
		{
			const int minJJminus1{ II * tileArraySizeIJ.JJ + (minJJ.at(II)-1) };
			if (minJJminus1 >= 0)
				mBoolmapAugmentedH.at(minJJminus1) = 1;
		}
		if (mBoolmapAugmentedH.at(II * tileArraySizeIJ.JJ + maxJJ.at(II)) == 1)
		{
			const int maxJJplus1{ II * tileArraySizeIJ.JJ + (maxJJ.at(II)+1) };
			if (maxJJplus1 < tileArraySizeIJ.II * tileArraySizeIJ.JJ)
				mBoolmapAugmentedH.at(maxJJplus1) = 1;
		}
	}

	//Combine the horizontal and vertical maps
	for (std::vector<int>::size_type iter = 0; iter != mBoolmapAugmentedV.size(); iter++)
		mBoolmap.at(iter) = mBoolmapAugmentedV.at(iter) || mBoolmapAugmentedH.at(iter);
}

void Boolmap::copyBoolmapToVector(std::vector<bool> &vec_input)
{
	vec_input.clear();
	for (std::vector<int>::size_type iter = 0; iter != mBoolmap.size(); iter++)
		vec_input.push_back(mBoolmap.at(iter));
}
#pragma endregion "Boolmap"



#pragma region "Stack"
Stack::Stack(const FFOV2 FFOV, const int tileHeight_pix, int const tileWidth_pix, const double pixelSizeZ, const int nFrames, const TILEOVERLAP3 overlapIJK_frac) :
	mFFOV{ FFOV },
	mTileHeight_pix{ tileHeight_pix },
	mTileWidth_pix{ tileWidth_pix },
	mPixelSizeZ{ pixelSizeZ },
	mDepthZ{ pixelSizeZ *  nFrames },
	mOverlapIJK_frac{ overlapIJK_frac }
{
	if (FFOV.XX <= 0 || FFOV.YY <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The FOV must be > 0");
	if (tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile height must be > 0");
	if (tileWidth_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel tile width must be > 0");
	if (pixelSizeZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel size Z must be > 0");
	if (nFrames <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The number of frames must be >= 1");
	if (overlapIJK_frac.II < 0 || overlapIJK_frac.JJ < 0 || overlapIJK_frac.II > 0.2 || overlapIJK_frac.JJ > 0.2 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in XY must be in the range [0-0.2]");
	if (overlapIJK_frac.KK < 0 || overlapIJK_frac.KK > 0.5)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack overlap in Z must be in the range [0-0.5]");
}

void Stack::printParams(std::ofstream *fileHandle) const
{
	*fileHandle << "STACK ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "FOV (stageX, stageY) = (" << mFFOV.XX / um << " um, " << mFFOV.YY / um << " um)\n";
	*fileHandle << "Pixel size Z = " << mPixelSizeZ / um << " um\n";
	*fileHandle << "Stack depth Z = " << mDepthZ / um << " um\n";
	*fileHandle << std::setprecision(2);
	*fileHandle << "Stack overlap = (" << mOverlapIJK_frac.II << ", " << mOverlapIJK_frac.JJ << ", " << mOverlapIJK_frac.KK << ") (frac)\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Stack overlap = (" << mOverlapIJK_frac.II * mFFOV.XX / um << " um, " << mOverlapIJK_frac.JJ * mFFOV.YY / um << " um, " << mOverlapIJK_frac.KK * mDepthZ << " um)\n";
	*fileHandle << "\n";
}

double Stack::readFFOV(const AXIS axis) const
{
	switch (axis)
	{
	case AXIS::XX:
		return mFFOV.XX;
	case AXIS::YY:
		return mFFOV.YY;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected stage axis unavailable");
	}
}

int Stack::readTileHeight_pix() const
{
	return mTileHeight_pix;
}

int Stack::readTileWidth_pix() const
{
	return mTileWidth_pix;
}

double Stack::readPixelSizeZ() const
{
	return mPixelSizeZ;
}

double Stack::readDepthZ() const
{
	return mDepthZ;
}

TILEOVERLAP3 Stack::readOverlapIJK_frac() const
{
	return mOverlapIJK_frac;
}

double Stack::readOverlap_frac(const TileArray::Axis axis) const
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
#pragma endregion "Stack"

#pragma region "Action"
void Action::MoveStage::setParam(const int sliceNumber, const TILEIJ tileIndicesIJ, const POSITION2 tileCenterXY)
{
	if (sliceNumber < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice number must be >= 0");
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.JJ < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile indices must be >= 0");

	mSliceNumber = sliceNumber;
	mTileIndicesIJ = tileIndicesIJ;
	mTileCenterXY = tileCenterXY;
}

int Action::MoveStage::readSliceNumber() const
{
	return mSliceNumber;
}

int Action::MoveStage::readTileIndex(const TileArray::Axis axis) const
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

POSITION2 Action::MoveStage::readTileCenterXY() const
{
	return mTileCenterXY;
}

double Action::MoveStage::readTileCenter(AXIS axis) const
{
	switch (axis)
	{
	case AXIS::XX:
		return mTileCenterXY.XX;
	case AXIS::YY:
		return mTileCenterXY.YY;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected stage axis unavailable");
	}
}

//stackNumber is the number of the stack following the scan pattern (e.g. snake). stackIndex is the number of the stack column by column from top to bottom and left to right (used for saving the tiff)
void Action::AcqStack::setParam(const int stackNumber, const int stackIndex, const int wavelength_nm, const SCANDIR scanDirZ, const double scanZmin, const double depthZ, const double scanPmin, const double scanPLexp, const int nFrameBinning)
{
	if (stackIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The stack number must be >= 0");
	if (wavelength_nm <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength must be > 0");
	if (scanZmin < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The Z stage height wavelength must be >= 0");
	if (depthZ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The scan depth must be > 0");
	if (scanPmin < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The laser power be >= 0");
	if (scanPLexp <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The power exponential constant must be > 0");
	if (nFrameBinning <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The binning number must be >= 1");

	mStackNumber = stackNumber;
	mStackIndex = stackIndex;
	mWavelength_nm = wavelength_nm;
	mScanDirZ = scanDirZ;
	mScanZmin = scanZmin;
	mDepthZ = depthZ;
	mScanPmin = scanPmin;
	mScanPLexp = scanPLexp;
	mNframeBinning = nFrameBinning;
}

int Action::AcqStack::readStackNumber() const
{
	return mStackNumber;
}

int Action::AcqStack::readStackIndex() const
{
	return mStackIndex;
}

int Action::AcqStack::readWavelength_nm() const
{
	return mWavelength_nm;
}

SCANDIR Action::AcqStack::readScanDirZ() const
{
	return mScanDirZ;
}

double Action::AcqStack::readScanZmin() const
{
	return mScanZmin;
}

double Action::AcqStack::readDepthZ() const
{
	return mDepthZ;
}

double Action::AcqStack::readScanPmin() const
{
	return mScanPmin;
}

double Action::AcqStack::readScanPLexp() const
{
	return mScanPLexp;
}

int Action::AcqStack::readNframeBinning() const
{
	return mNframeBinning;
}

void Action::CutSlice::setParam(const double planeZtoCut, const double stageZheightForFacingTheBlade)
{
	if (planeZtoCut < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The Z stage height must be >= 0");
	if (stageZheightForFacingTheBlade < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The Z stage height must be >= 0");

	mPlaneZtoCut = planeZtoCut;
	mStageZheightForFacingTheBlade = stageZheightForFacingTheBlade;
}

double Action::CutSlice::readPlaneZtoCut() const
{
	return mPlaneZtoCut;
}

double Action::CutSlice::readStageZheightForFacingTheBlade() const 
{
	return mStageZheightForFacingTheBlade;
}

void Action::PanoramicScan::setParam(const int sliceNumber, const double planeZ)
{
	if (sliceNumber < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The slice number must be >= 0");
	if (planeZ < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The Z stage height must be >= 0");

	mSliceNumber = sliceNumber;
	mPlaneZ = planeZ + 50. * um;
}

double Action::PanoramicScan::readPlaneZ() const
{
	return mPlaneZ;
}

int Action::PanoramicScan::readSliceNumber() const
{
	return mSliceNumber;
}
#pragma endregion "Action"

#pragma region "Commandline"
Sequencer::Commandline::Commandline(const Action::ID actionID) :
	mActionID{ actionID }
{}

void Sequencer::Commandline::printToFile(std::ofstream *fileHandle) const
{
	switch (mActionID)
	{
	case Action::ID::MOV:
		*fileHandle << convertActionIDtoString_(mActionID) << "\t" << mAction.moveStage.readSliceNumber();
		*fileHandle << "\t(" << mAction.moveStage.readTileIndex(TileArray::Axis::II) << "," << mAction.moveStage.readTileIndex(TileArray::Axis::JJ) << ")\t";
		*fileHandle << std::setprecision(4);
		*fileHandle << "(" << mAction.moveStage.readTileCenter(AXIS::XX) / mm << "," << mAction.moveStage.readTileCenter(AXIS::YY) / mm << ")\n";
		break;
	case Action::ID::ACQ:
		*fileHandle << convertActionIDtoString_(mActionID) << "\t\t\t\t\t";
		*fileHandle << mAction.acqStack.readStackIndex() << "\t";
		*fileHandle << mAction.acqStack.readWavelength_nm() << "\t";
		*fileHandle << std::setprecision(3);
		*fileHandle << mAction.acqStack.readScanZmin() / mm << "\t";
		*fileHandle << (mAction.acqStack.readScanZmin() + mAction.acqStack.readDepthZ()) / mm << "\t";
		*fileHandle << std::setprecision(0);
		*fileHandle << mAction.acqStack.readScanPmin() << "\t" << mAction.acqStack.readScanPLexp() << "\n";
		break;
	case Action::ID::SAV:
		*fileHandle << convertActionIDtoString_(mActionID) + "\n";
		break;
	case Action::ID::CUT:
		*fileHandle << convertActionIDtoString_(mActionID);
		*fileHandle << std::setprecision(3);
		*fileHandle << "******Stage height for facing the VT = " << mAction.cutSlice.readStageZheightForFacingTheBlade() / mm << " mm";
		*fileHandle << "******Equivalent sample plane Z = " << mAction.cutSlice.readPlaneZtoCut() / mm << " mm";
		*fileHandle << "********\n";
		break;
	case Action::ID::PAN:
		*fileHandle << convertActionIDtoString_(mActionID);
		*fileHandle << std::setprecision(3);
		*fileHandle << "******************************Panoramic scan at plane z = " << mAction.panoramicScan.readPlaneZ() / mm << " mm";
		*fileHandle << "********************************\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
/*
void Sequencer::Commandline::printParameters() const
{
	switch (mActionID)
	{
	case Action::ID::MOV:
		std::cout << "The command is " << convertActionIDtoString_(mActionID) << " with parameters: \n";
		std::cout << "Vibratome slice number = " << mAction.moveStage.readSliceNumber() << "\n";
		std::cout << "Tile ij = (" << mAction.moveStage.readTileIndex(TileArray::Axis::II) << "," << mAction.moveStage.readTileIndex(TileArray::Axis::JJ) << ")\n";
		std::cout << "Tile center (stageX, stageY) = (" << mAction.moveStage.readTileCenter(AXIS::XX) / mm << "," << mAction.moveStage.readTileCenter(AXIS::YY) / mm << ") mm\n\n";
		break;
	case Action::ID::ACQ:
		std::cout << "The command is " << convertActionIDtoString_(mActionID) << " with parameters: \n";
		std::cout << "wavelength = " << mAction.acqStack.readWavelength_nm() << " nm\n";
		std::cout << "scanDirZ = " << Util::convertScandirToInt(mAction.acqStack.readScanDirZ()) << "\n";
		std::cout << "scanZmin / depthZ = " << mAction.acqStack.readScanZmin() / mm << " mm/" << mAction.acqStack.readDepthZ() << " mm\n";
		std::cout << "scanPmin / ScanPLexp = " << mAction.acqStack.readScanPmin() / mW << " mW/" << mAction.acqStack.readScanPLexp() / um << " (um)\n\n";
		break;
	case Action::ID::SAV:
		std::cout << "The command is " << convertActionIDtoString_(mActionID) << "\n";
		break;
	case Action::ID::CUT:
		std::cout << "The command is " << convertActionIDtoString_(mActionID) << "\n";
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
*/

std::string Sequencer::Commandline::convertActionIDtoString_(const Action::ID actionID) const
{
	switch (actionID)
	{
	case Action::ID::CUT:
		return "CUT";
	case Action::ID::SAV:
		return "SAV";
	case Action::ID::ACQ:
		return "ACQ";
	case Action::ID::MOV:
		return "MOV";
	case Action::ID::PAN:
		return "PAN";
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
	}
}
#pragma endregion "Commandline"

#pragma region "Sequencer"
//Constructor using the sample's LOI. The number of stacks is calculated automatically based on the FFOV
Sequencer::Sequencer(const Sample sample, const Stack stack) :
	mSample{ sample },
	mStack{ stack },
	mTileArray{ stack.readTileHeight_pix(), stack.readTileWidth_pix(), determineTileArraySizeIJ_(), stack.readOverlapIJK_frac() }
{
	initializeVibratomeSlice_();
	initializeEffectiveROI_();		//Calculate the effective ROI covered by all the tiles, which might be slightly larger than the requested ROI
	reserveMemoryBlock_();			//Reserve memory for speed
}

//Generate a scan pattern and use the vibratome to slice the sample
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::generateCommandList()
{
	std::cout << "Generating the command list..." << "\n";
	for (int iterSlice = 0; iterSlice < mNtotalSlices; iterSlice++)
	{
		panoramicScan_();

		initializeIteratorIJ_();		//Reset the tile iterator after every cut
		//The first fluor-marker on the list is read, then the second marker, etc
		for (std::vector<int>::size_type iterFluorMarker = 0; iterFluorMarker != mSample.readFluorMarkerListSize(); iterFluorMarker++)
		{
			//The Y-stage is the slowest to react because it sits under the 2 other stages. For the best performance, iterate over II often and over JJ less often
			while (mJJ >= 0 && mJJ < mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))		//Y-stage iteration
			{
				while (mII >= 0 && mII < mTileArray.readTileArraySizeIJ(TileArray::Axis::II))	//X-stage iteration
				{
					moveStage_({ mII, mJJ });
					acqStack_(iterFluorMarker);
					saveStack_();
					mII -= Util::convertScandirToInt(mIterScanDirXYZ.XX);	//Increase/decrease the iterator in the X-stage axis
				}
				mII += Util::convertScandirToInt(mIterScanDirXYZ.XX);		//Re-initialize II by going back one step to start from 0 or mTileArraySizeIJ.II - 1
				reverseSCANDIR(mIterScanDirXYZ.XX);							//Reverse the scanning direction
				mJJ -= Util::convertScandirToInt(mIterScanDirXYZ.YY);		//Increase/decrease the iterator in the Y-stage axis
			}
			mJJ += Util::convertScandirToInt(mIterScanDirXYZ.YY);			//Re-initialize JJ by going back one step to start from 0 or mTileArraySizeIJ.JJ - 1
			reverseSCANDIR(mIterScanDirXYZ.YY);								//Reverse the scanning direction
		}
		//Only need to cut the sample 'nVibratomeSlices -1' times
		if (iterSlice < mNtotalSlices - 1)
			cutSlice_();
	}
}

//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
//The center the tile array is at the sample center (exact center for an odd number of tiles or slightly off center for an even number of tiles)
POSITION2 Sequencer::convertTileIndicesIJToStagePosXY(const TILEIJ tileIndicesIJ) const
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	//The tile centers are (1-a)*FFOV away from each other, where a*L is the tile overlap
	POSITION2 centeredIJ{ determineRelativeTileIndicesIJ(mStack.readOverlapIJK_frac(), mTileArray.readTileArraySizeIJ(), tileIndicesIJ) };
	POSITION2 stagePosXY;
	stagePosXY.XX = mSample.mCenterXY.XX - mStack.readFFOV(AXIS::XX)  * centeredIJ.XX;
	stagePosXY.YY = mSample.mCenterXY.YY - mStack.readFFOV(AXIS::YY)  * centeredIJ.YY;

	return stagePosXY;
}

std::string Sequencer::printHeader() const
{
	return 	"Action\tSlice#\tTileIJ\t(stageX,stageY)\tStackID\tWavlen\tstageZ<\tstageZ>\tPmin\tPLexp";
}

std::string Sequencer::printHeaderUnits() const
{
	return "\t\t(mm,mm)\t\t\tnm\tmm\tmm\tmW\tum";
}

void Sequencer::printSequenceParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SEQUENCER ************************************************************\n";
	*fileHandle << "Stages initial scan direction {stageX, stageY, stageZ} = {" << Util::convertScandirToInt(g_initialStageScanDirXYZ.XX) << ", " <<
																				   Util::convertScandirToInt(g_initialStageScanDirXYZ.YY) << ", " <<
																				   Util::convertScandirToInt(g_initialStageScanDirXYZ.ZZ) << "}\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Effective LOI (stageX, stageY, stageZ) = (" << determineEffectiveLOIxyz().XX / mm << " mm, " <<
																   determineEffectiveLOIxyz().YY / mm << " mm, " <<
																   determineEffectiveLOIxyz().ZZ / mm << " mm)\n";

	*fileHandle << "Effective ROI boundaries [YMIN, XMIN, YMAX, XMAX] = [" << mROI.YMIN / mm << " mm, " <<
																			  mROI.XMIN / mm << " mm, " <<
																			  mROI.YMAX / mm << " mm, " <<
																			  mROI.XMAX / mm << " mm]\n";
	*fileHandle << "Z position of the surface of the sample = " << mSample.mSurfaceZ / mm << " mm\n";
	*fileHandle << std::setprecision(0);
	*fileHandle << "Total # tissue slices = " << mNtotalSlices << "\n";
	*fileHandle << "Tile array size (stageX, stageY) = (" << mTileArray.readTileArraySizeIJ(TileArray::Axis::II) << ", " <<
															 mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) << ")\n";

	*fileHandle << "Total # stacks entire sample = " << mStackCounter << "\n";
	*fileHandle << "Total # commandlines = " << mCommandCounter << "\n";

	const double imagingTimePerStack{ g_lineclockHalfPeriod *  mStack.readTileHeight_pix() * (mStack.readDepthZ() / mStack.readPixelSizeZ()) / 16 };
	const double totalImagingTime_hours{ (mStackCounter + 1) * imagingTimePerStack / seconds / 3600. };

	*fileHandle << "Runtime per stack = " << imagingTimePerStack / ms << " ms (times nBinning)\n";
	*fileHandle << std::setprecision(6);
	*fileHandle << "Estimated total runtime (multibeam + pipelining) = " << totalImagingTime_hours << " hrs (times nBinning)\n\n";
}

//Print the commandlist to file
void Sequencer::printToFile(std::string filename, const OVERRIDE override) const
{
	if (override == OVERRIDE::DIS)
		filename = Util::doesFileExist(filename, ".txt");

	std::ofstream *fileHandle{ new std::ofstream(g_folderPath + filename + ".txt") };

	*fileHandle << std::fixed;	//Show a fixed number of digits

	mSample.printSampleParams(fileHandle);
	mSample.printFluorParams(fileHandle);
	printSequenceParams(fileHandle);
	mStack.printParams(fileHandle);

	//Print out the header
	if (!mCommandList.empty())
	{
		*fileHandle << "Act#\t" + printHeader() + "\n";
		*fileHandle << "\t\t" + printHeaderUnits() + "\n";
	}

	for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != mCommandList.size(); iterCommandline++)
		//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 3*(3 * 3 * mTileArraySizeIJ.at(X) * mTileArraySizeIJ.at(Y)+1) + 2 ; iterCommandline++) //For debugging
	{
		*fileHandle << iterCommandline << "\t";		//Print out the iteration number
		mCommandList.at(iterCommandline).printToFile(fileHandle);
	}

	*fileHandle << std::defaultfloat;
	fileHandle->close();
}

int Sequencer::readNumberOfStacksPerSlice() const
{
	return mTileArray.readTileArraySizeIJ(TileArray::Axis::II) * mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ);
}

int Sequencer::readTotalNumberOfSlices() const
{
	return mNtotalSlices;
}

int Sequencer::readTotalNumberOfStacks() const
{
	return mStackCounter;
}

int Sequencer::readNtotalCommands() const
{
	return mCommandCounter;
}

Sequencer::Commandline Sequencer::readCommandline(const int iterCommandLine) const
{
	return mCommandList.at(iterCommandLine);
}

int Sequencer::readTileArraySizeIJ(const TileArray::Axis axis) const
{
	return mTileArray.readTileArraySizeIJ(axis);
}

TILEDIM2 Sequencer::readTileArraySizeIJ() const
{
	return mTileArray.readTileArraySizeIJ();
}

void Sequencer::initializeVibratomeSlice_()
{
	mIterScanZi = mSample.mSurfaceZ; 	//Initialize the Z-stage with the position of the surface of the sample

	mIterSamplePlaneZtoCut = mIterScanZi + mStack.readDepthZ() - mSample.mCutAboveBottomOfStack;
	mIterStageZheightForFacingTheBlade = mIterSamplePlaneZtoCut + mSample.mBladeFocalplaneOffsetZ;

	const int numberOfVibratomeSlices{ static_cast<int>(1 + std::ceil(1. / (1 - mStack.readOverlap_frac(TileArray::Axis::KK)) * (mSample.mLOIxyz_req.ZZ / mStack.readDepthZ() - 1))) };	//Total number of vibratome slices in the entire sample
	if (numberOfVibratomeSlices > 1)
		mNtotalSlices = numberOfVibratomeSlices;		
	else
		mNtotalSlices = 1;
}

//Calculate the number of tiles in the X-stage and Y-stage axes based on the length of interest (LOI) and the FFOV
//If the overlap between consecutive tiles is a*FOV, then N tiles cover the distance L = FOV * ( (1-a)*(N-1) + 1 ), thus N = 1/(1-a) * ( L/FOV - 1 ) + 1
TILEDIM2 Sequencer::determineTileArraySizeIJ_()
{
	const int numberOfTilesII{ Util::intceil(1 + 1. / (1 - mStack.readOverlap_frac(TileArray::Axis::II)) * (mSample.mLOIxyz_req.XX / mStack.readFFOV(AXIS::XX) - 1)) };
	const int numberOfTilesJJ{ Util::intceil(1 + 1. / (1 - mStack.readOverlap_frac(TileArray::Axis::JJ)) * (mSample.mLOIxyz_req.YY / mStack.readFFOV(AXIS::YY) - 1)) };

	TILEDIM2 output;
	if (numberOfTilesII > 1)
		output.II = numberOfTilesII;		//Number of tiles in the X-stage axis
	else
		output.II = 1;

	if (numberOfTilesJJ > 1)
		output.JJ = numberOfTilesJJ;		//Number of tiles in the Y-stage axis	
	else
		output.JJ = 1;

	//For debugging
	//std::cout << "Number of row tiles = " << numberOfTilesII << "\tNumber of column tiles = " << numberOfTilesJJ << "\n";

	return output;
}

//Calculate the effective ROI =  { YMIN, XMIN, YMAX, XMAX } covered by all the tiles
void Sequencer::initializeEffectiveROI_()
{
	//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
	const POSITION2 tilePosXYmin = convertTileIndicesIJToStagePosXY({ 0, 0 });																												//Absolute position of the CENTER of the tile
	const POSITION2 tilePosXYmax = convertTileIndicesIJToStagePosXY({ mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1, mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1 });	//Absolute position of the CENTER of the tile

	//The ROI is measured from the border of the tiles. Therefore, add half of the FFOV
	mROI.XMAX = tilePosXYmin.XX + mStack.readFFOV(AXIS::XX) / 2.;
	mROI.YMAX = tilePosXYmin.YY + mStack.readFFOV(AXIS::YY) / 2.;
	mROI.XMIN = tilePosXYmax.XX - mStack.readFFOV(AXIS::XX) / 2.;
	mROI.YMIN = tilePosXYmax.YY - mStack.readFFOV(AXIS::YY) / 2.;

	if (mROI.XMIN < mSample.readStageSoftPosLimXMIN() || mROI.XMAX > mSample.readStageSoftPosLimXMAX())
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The ROI goes beyond the soft limits of the stage X");
	if (mROI.YMIN < mSample.readStageSoftPosLimYMIN() || mROI.YMAX >  mSample.readStageSoftPosLimYMAX())
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The ROI goes beyond the soft limits of the stage Y");
}

//Reserve a memory block assuming 3 actions for every stack in each vibratome slice: MOV, ACQ, and SAV. Then CUT the slice
void Sequencer::reserveMemoryBlock_()
{
	const int nTotalTilesPerVibratomeSlice{ mTileArray.readTileArraySizeIJ(TileArray::Axis::II) * mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) };	//Total number of tiles in a vibratome slice
	const int nTotalTilesEntireSample{ mNtotalSlices * static_cast<int>(mSample.readFluorMarkerListSize()) * nTotalTilesPerVibratomeSlice };				//Total number of tiles in the entire sample. mNtotalSlices is fixed at 1
	mCommandList.reserve(3 * nTotalTilesEntireSample + mNtotalSlices - 1);
}

//Reset the iterators mII and mJJ to the initial values
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::initializeIteratorIJ_()
{
	resetStageScanDirections_();	//Reset the scan directions

	switch (mIterScanDirXYZ.XX)
	{
	case SCANDIR::RIGHTWARD:	//The X-stage moves to the right, therefore, the sample is imaged from right to left
		mII = mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1;
		break;
	case SCANDIR::LEFTWARD:		//The X-stage moves to the left, therefore, the sample is imaged from left to right
		mII = 0;
		break;
	}

	switch (mIterScanDirXYZ.YY)
	{
	case SCANDIR::INWARD:		//The Y-stage moves inward, therefore, the sample is imaged from "inside" to "outside" of the microscope
		mJJ = mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1;
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
LENGTH3 Sequencer::determineEffectiveLOIxyz() const
{
	return { mROI.XMAX - mROI.XMIN,
			 mROI.YMAX - mROI.YMIN,
			 mStack.readDepthZ() * ((1 - mStack.readOverlap_frac(TileArray::Axis::KK)) * (mNtotalSlices - 1) + 1) };
}

//Move the stage to the position corresponding to the tile indices II and JJ 
//II is the row index (along the image height and X-stage) and JJ is the column index (along the image width and Y-stage) of the tile. II and JJ start from 0
void Sequencer::moveStage_(const TILEIJ tileIndicesIJ)
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArray.readTileArraySizeIJ(TileArray::Axis::II))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index II must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::II) - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ))
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile index JJ must be in the range [0-" + std::to_string(mTileArray.readTileArraySizeIJ(TileArray::Axis::JJ) - 1) + "]");

	const POSITION2 tileCenterXY = convertTileIndicesIJToStagePosXY(tileIndicesIJ);

	Commandline commandline{ Action::ID::MOV };
	commandline.mAction.moveStage.setParam(mSliceCounter, tileIndicesIJ, tileCenterXY);
	mCommandList.push_back(commandline);
	mCommandCounter++;	//Count the number of commands
}

//indexFluorMarker is the position of the fluorMarker in mFluorMarkerList
void Sequencer::acqStack_(const int indexFluorMarker)
{
	if (indexFluorMarker < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The wavelength index must be >= 0");

	//Read the corresponding laser configuration
	const FluorMarkerList::FluorMarker fluorMarker{ mSample.readFluorMarker(indexFluorMarker) };

	//The stages scan through a snake pattern but the saving is column by column
	//Stack index for identifying the saved tiff file
	const int stackIndex{ mJJ * readTileArraySizeIJ(TileArray::Axis::II) + mII };

	Commandline commandline{ Action::ID::ACQ };
	commandline.mAction.acqStack.setParam(mStackCounter, stackIndex, fluorMarker.mWavelength_nm, mIterScanDirXYZ.ZZ, mIterScanZi, mStack.readDepthZ(), fluorMarker.mScanPmin, fluorMarker.mScanPLexp, fluorMarker.nFramesBinning);
	mCommandList.push_back(commandline);
	mStackCounter++;	//Count the number of stacks
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
	Commandline commandline{ Action::ID::CUT };
	commandline.mAction.cutSlice.setParam(mIterSamplePlaneZtoCut, mIterStageZheightForFacingTheBlade);
	mCommandList.push_back(commandline);
	mSliceCounter++;	//Count the number of vibratome slices
	mCommandCounter++;	//Count the number of commands

	//Increase the height of the Z-stage and the height of the plane to cut in the next iteration		
	//Because of the overlap, the effective stack depth is (1-a)*stackDepth, where a*stackDepth is the overlap
	const double heightIncrease{ (1 - mStack.readOverlap_frac(TileArray::Axis::KK)) * mStack.readDepthZ() };

	mIterScanZi += heightIncrease;
	mIterSamplePlaneZtoCut += heightIncrease;
	mIterStageZheightForFacingTheBlade += heightIncrease;
}

void Sequencer::panoramicScan_()
{
	Commandline commandline{ Action::ID::PAN };
	commandline.mAction.panoramicScan.setParam(mSliceCounter, mIterScanZi);
	mCommandList.push_back(commandline);
	mCommandCounter++;
}
#pragma endregion "sequencer"