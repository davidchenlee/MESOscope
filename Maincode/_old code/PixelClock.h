#pragma once
#include "Const.h"
#include "FPGAapi.h"

class PixelClock
{
private:
	U32Q Queue;
	const int latency_tick = 2;		//latency of detecting the line clock. Calibrate the latency with the oscilloscope
	double ConvertSpatialCoord2Time(double x);
	double getDiscreteTime(int pix);
	double calculateDwellTime(int pix);
	double calculatePracticalDwellTime(int pix);

public:
	PixelClock();
	~PixelClock();
	U32Q PixelClockEqualDuration();
	U32Q PixelClockEqualDistance();
};

