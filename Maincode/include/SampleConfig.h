#pragma once
#include <Const.h>
#include "Utilities.h"
using namespace Constants;

const extern std::vector<LIMIT2> PetridishPosLimit;
const extern std::vector<LIMIT2> ContainerPosLimit;
const extern POSITION3 g_stackCenterXYZ;

class FluorMarkerList						//Create a list of fluorescent markers
{
public:
	struct FluorMarker						//Parameters for a single fluorescent marker
	{
		std::string mName{ "" };			//Fluorescent marker name
		int mWavelength_nm;					//Laser wavelength
		double mScanPmin;					//Initial laser power for a stack scan. It could be >= or <= than the final laser power depending on the scan direction
		double mScanPLexp{ 5000. * um };	//Length constant for the exponential power increase
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
	//Should I initialize the members with std::numeric_limits<double>::quiet_NaN?
	POSITION2 mCenterXY{ -1, -1 };						//Sample center (stageX, stageY)
	LENGTH3 mLOIxyz_req{ -1, -1, -1 };					//Requested Length of interest (stageX, stageY, stageZ)
	double mSurfaceZ{ -1 };
	const double mBladeFocalplaneOffsetZ{ 1.620 * mm };	//Positive distance if the blade is higher than the microscope's focal plane; negative otherwise//1.810 mm
	double mCutAboveBottomOfStack{ 0. * um };			//Specify at what height of the overlapping volume to cut

	Sample(const std::string sampleName, const std::string immersionMedium, const std::string objectiveCollar, const std::vector<LIMIT2> stageSoftPosLimXYZ, const FluorMarkerList fluorMarkerList = { {} });
	Sample(const Sample& sample, const POSITION2 centerXY, const LENGTH3 LOIxyz, const double sampleSurfaceZ, const double sliceOffset);
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

const extern Sample g_currentSample;
