#pragma once
#include "Const.h"
#include "FPGAapi.h"

class PixelClock
{
private:
	U32Q Queue;
	const int latency_tick = 2;		//latency of detecting the line clock. Calibrate the latency with the oscilloscope
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

