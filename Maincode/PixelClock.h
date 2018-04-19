#pragma once
#include "Const.h"
#include "FPGAlowlevel.h"

class PixelClock
{
private:
	std::vector<double> PixelClockEqualDistanceLUT;
	double PixelClock::ConvertSpatialCoord2Time(double x);
	double PixelClock::getDiscreteTime(int pix);
	double PixelClock::calculateDwellTime(int pix);
	double PixelClock::calculatePracticalDwellTime(int pix);

public:
	PixelClock::PixelClock();
	U32Q PixelClock::PixelClockEqualDuration();
	U32Q PixelClock::PixelClockEqualDistance();
	~PixelClock();
};

