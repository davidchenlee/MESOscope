#include "SampleConfig.h"

#pragma region "FluorMarkerList"
FluorMarkerList::FluorMarkerList(const std::vector<FluorMarker> fluorMarkerList) :
	mFluorMarkerList{ fluorMarkerList }
{}

std::size_t FluorMarkerList::readFluorMarkerListSize() const
{
	return mFluorMarkerList.size();
}

void FluorMarkerList::printFluorParams(std::ofstream *fileHandle) const
{
	*fileHandle << "LASERS ************************************************************\n";

	for (std::vector<int>::size_type iterWL = 0; iterWL != mFluorMarkerList.size(); iterWL++)
	{
		*fileHandle << "Wavelength = " << mFluorMarkerList.at(iterWL).mWavelength_nm <<
			" nm\nPower = " << mFluorMarkerList.at(iterWL).mScanPmin / mW <<
			" mW\nPower exponential length = " << mFluorMarkerList.at(iterWL).mScanPexp / um << " um\n";
	}
	*fileHandle << "\n";
}

//Return the first instance of "fluorMarker" in mFluorMarkerList
FluorMarkerList::FluorMarker FluorMarkerList::findFluorMarker(const std::string fluorMarker) const
{
	for (std::vector<int>::size_type iterMarker = 0; iterMarker < mFluorMarkerList.size(); iterMarker++)
	{
		if (!fluorMarker.compare(mFluorMarkerList.at(iterMarker).mName)) //compare() returns 0 if the strings are identical
			return mFluorMarkerList.at(iterMarker);
	}
	//If the requested fluorMarker is not found
	throw std::runtime_error((std::string)__FUNCTION__ + ": Fluorescent marker " + fluorMarker + " not found");
}

//indexFluorMarker is the position of the fluorMarker in mFluorMarkerList
FluorMarkerList::FluorMarker FluorMarkerList::readFluorMarker(const int indexFluorMarker) const
{
	return mFluorMarkerList.at(indexFluorMarker);
}

FluorMarkerList FluorMarkerList::readFluorMarkerList() const
{
	return mFluorMarkerList;
}
#pragma endregion "FluorMarkerList"

#pragma region "Sample"
Sample::Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorMarkerList fluorMarkerList) :
	mName{ sampleName },
	mImmersionMedium{ immersionMedium },
	mObjectiveCollar{ objectiveCollar },
	mStageSoftPosLimXYZ{ stageSoftPosLimXYZ },
	FluorMarkerList{ fluorMarkerList }
{}

Sample::Sample(const Sample& sample, const POSITION2 centerXY, const LENGTH3 LOIxyz, const double sampleSurfaceZ, const double sliceOffset) :
	mName{ sample.mName },
	mImmersionMedium{ sample.mImmersionMedium },
	mObjectiveCollar{ sample.mObjectiveCollar },
	mStageSoftPosLimXYZ{ sample.mStageSoftPosLimXYZ },
	FluorMarkerList{ sample.readFluorMarkerList() },
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

void Sample::printSampleParams(std::ofstream *fileHandle) const
{
	*fileHandle << "SAMPLE ************************************************************\n";
	*fileHandle << "Name = " << mName << "\n";
	*fileHandle << "Immersion medium = " << mImmersionMedium << "\n";
	*fileHandle << "Correction collar = " << mObjectiveCollar << "\n";
	*fileHandle << std::setprecision(4);
	*fileHandle << "Sample center (stageX, stageY) = (" << mCenterXY.XX / mm << " mm, " << mCenterXY.YY / mm << " mm)\n";
	*fileHandle << "Requested ROI size (stageX, stageY, stageZ) = (" << mLOIxyz_req.XX / mm << " mm, " << mLOIxyz_req.YY / mm << " mm, " << mLOIxyz_req.ZZ / mm << " mm)\n\n";

	*fileHandle << "SLICE ************************************************************\n";
	*fileHandle << std::setprecision(1);
	*fileHandle << "Blade-focal plane vertical offset = " << mBladeFocalplaneOffsetZ / um << " um\n";
	*fileHandle << "Cut above the bottom of the stack = " << mCutAboveBottomOfStack / um << " um\n";
	*fileHandle << "\n";
}

std::string Sample::readName() const
{
	return mName;

}

std::string Sample::readImmersionMedium() const
{
	return mImmersionMedium;
}

std::string Sample::readObjectiveCollar() const
{
	return mObjectiveCollar;
}

std::vector<LIMIT2> Sample::readStageSoftPosLimXYZ() const
{
	return mStageSoftPosLimXYZ;
}

double Sample::readStageSoftPosLimXMIN() const
{
	return mStageSoftPosLimXYZ.at(Axis::XX).MIN;
}

double Sample::readStageSoftPosLimXMAX() const
{
	return mStageSoftPosLimXYZ.at(Axis::XX).MAX;
}

double Sample::readStageSoftPosLimYMIN() const
{
	return mStageSoftPosLimXYZ.at(Axis::YY).MIN;
}

double Sample::readStageSoftPosLimYMAX() const
{
	return mStageSoftPosLimXYZ.at(Axis::YY).MAX;
}
#pragma endregion "Sample"

const extern std::vector<LIMIT2> PetridishPosLimit{ { 27. * mm, 57. * mm}, { 0. * mm, 30. * mm}, { 15. * mm, 24. * mm} };		//Soft limit of the stage for the petridish
const extern std::vector<LIMIT2> ContainerPosLimit{ { -65. * mm, 65. * mm}, { 1.99 * mm, 30. * mm}, { 10. * mm, 24. * mm} };	//Soft limit of the stage for the oil container

//SAMPLE PARAMETERS
//const extern POSITION3 g_stackCenterXYZ{ (44.300 + 1.456) * mm, (24.003 + 9.904/2 - 0.285)* mm, (17.840 + 0.000) * mm };
//const extern POSITION3 g_stackCenterXYZ{ (44.300) * mm, (24.003)* mm, (18.051 + 0.000) * mm };//For contScanX
const extern POSITION3 g_stackCenterXYZ{ (53.360 - 0.035) * mm, (25.000 - 0.017)* mm, (17.920) * mm };

#if multibeam
const extern Sample g_currentSample{ "Beads4um16X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, multiply16X(30. * mW), multiply16X(1000. * um) },
																				   { "GFP", 920, multiply16X(30. * mW), multiply16X(1000. * um) },
																				   { "TDT", 1040, multiply16X(15. * mW), multiply16X(1000. * um) } }} };
/*const extern Sample liver{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{ {"TDT", 1040, multiply16X(50. * mW), multiply16X(0.0) },
																							{ "GFP", 920, multiply16X(40. * mW), multiply16X(0.0) },
																							{ "DAPI", 750, multiply16X(50. * mW), multiply16X(0.) } }} };*/
																							/*Sample g_currentSample{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit, {{ {"TDT", 1040, multiply16X(50. * mW), 150., 4 },
																																																  { "DAPI", 750, multiply16X(20. * mW), 120., 2 } }} };*/
#else
/*const extern Sample g_currentSample{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit,  {{{"TDT", 1040, 30. * mW, 150. * um, 4 },
																									  { "DAPI", 750, 12. * mW, 120. * um, 2 }}} };*/

const extern Sample g_currentSample{ "Beads4um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 20. * mW, 1000. * um},
																				  { "GFP", 920, 20. * mW, 1000. * um},
																				  { "TDT", 1040, 10. * mW, 1000. * um}}} };
/*const extern Sample g_currentSample{ "Beads1um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 40. * mW, 0. },
																					{ "GFP", 920, 40. * mW, 0. },
																					{ "TDT", 1040, 15. * mW, 0. }}} };*/
																					/*Sample g_currentSample{ "fluorBlue1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{ "DAPI", 750, 10. * mW, 0. }}} };*/
#endif