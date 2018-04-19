#include "PixelClock.h"

PixelClock::PixelClock(): PixelClockEqualDistanceLUT(WidthPerFrame_pix,0)
{

	if (WidthPerFrame_pix % 2 == 0)	//is even
	{
		for (int pix = -WidthPerFrame_pix / 2; pix < WidthPerFrame_pix / 2; pix++)	//pix in [-WidthPerFrame_pix/2,WidthPerFrame_pix/2]
			PixelClockEqualDistanceLUT[pix + WidthPerFrame_pix / 2] = calculatePracticalDwellTime(pix);
	}
	else
	{
		std::cerr << "ERROR in " << __func__ << ": Odd number of pixels in the image width currently not supported by the pixel clock. Pixel clock set to 0" << std::endl;
	}
}

//Pixel clock sequence. Every pixel has the same duration in time.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_us'. At 160MHz, the clock increment is 6.25ns = 0.00625us
//Pixel clock evently spaced in time
U32Q PixelClock::PixelClockEqualDuration()
{
	U32Q Q;																//Create a queue

	const double InitialWaitingTime_us = 6.25*us;						//Initial waiting time to center the pixel clock in a line scan
																		//Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us. Therefore, the waiting time is (62.5-50)/2 = 6.25us
	int latency_tick = 2;												//latency of detecting the line clock. Calibrate the latency with the oscilloscope
	Q.push(packU32(convertUs2tick(InitialWaitingTime_us) - latency_tick, 0x0000));

	const double PixelTimeStep = 0.125 * us;
	for (int pix = 0; pix < WidthPerFrame_pix + 1; pix++)
		Q.push(generateSinglePixelClock(PixelTimeStep, 1));				//Generate the pixel clock. Every time 1 is pushed, the pixel clock "ticks" (flips its state), which serves as a pixel delimiter
																		//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant
	return Q;															//Return a queue (and not a vector of queues)
}

//Pixel clock sequence. Every pixel is equally spaced.
//The pixel clock is triggered by the line clock (see the LV implementation), followed by a waiting time 'InitialWaitingTime_tick'. At 160MHz, the clock increment is 6.25ns = 0.00625us
U32Q PixelClock::PixelClockEqualDistance()
{
	U32Q Q;																		//Create a queue

	const U16 InitialWaitingTime_tick = 2043;									//Initial waiting time. Look at the oscilloscope and adjust this parameter to center the pixel clock in a line scan
	int latency_tick = 2;														//Latency of detecting the line clock. Calibrate the latency with the oscilloscope
	Q.push(packU32(InitialWaitingTime_tick - latency_tick, 0x0000));

	for (int pix = 0; pix < WidthPerFrame_pix; pix++)
		Q.push(generateSinglePixelClock(PixelClockEqualDistanceLUT[pix], 1));	//Generate the pixel clock.Every time 1 is pushed, the pixel clock "ticks" (flips its state), which serves as a pixel delimiter

	Q.push(generateSinglePixelClock(dt_us_MIN, 1));								//Npixels+1 because there is one more pixel delimiter than number of pixels. The last time step is irrelevant

	return Q;
}

PixelClock::~PixelClock()
{
}


//Convert the spatial coordinate of the resonant scanner to time. x in [-RSamplitudePkPK_um/2, RSamplitudePkPK_um/2]
double PixelClock::ConvertSpatialCoord2Time(double x)
{
	const double arg = 2 * x / RSamplitudePkPK_um;
	if (arg <= 1)
		return HalfPeriodLineClock_us * asin(arg) / PI; //Return value in [-HalfPeriodLineClock_us/PI, HalfPeriodLineClock_us/PI]
	else
	{
		std::cerr << "ERROR in " << __func__ << ": argument of asin greater than 1" << std::endl;
		return HalfPeriodLineClock_us / PI;
	}
}

//Discretize the spatial coordinate, then convert it to time
double PixelClock::getDiscreteTime(int pix)
{
	const double dx = 0.5 * um;
	return ConvertSpatialCoord2Time(dx * pix);
}

//Calculate the dwell time for the pixel
double PixelClock::calculateDwellTime(int pix)
{
	return getDiscreteTime(pix + 1) - getDiscreteTime(pix);
}

//Calculate the dwell time of the pixel but considering that the FPGA has a finite clock rate
double PixelClock::calculatePracticalDwellTime(int pix)
{
	return round(calculateDwellTime(pix) * tickPerUs) / tickPerUs;		// 1/tickPerUs is the time step of the FPGA clock (microseconds per tick)
}
