#include "FPGAsequences.h"

#pragma region "FPGA combined sequences"

void FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session)
{
	//control sequences
	SendOutQueue(status, session, Acquire2D());
	TriggerFIFOIN(status, session);			//trigger the analog and digital outputs

											//TriggerAcquisition(status, session); // trigger the data acquisition
	CountPhotons(status, session);


	//SECOND ROUND
	if (0)
	{
		//SendOutQueue(status, session, TestAODO());
		TriggerFIFOIN(status, session);
		TriggerAcquisition(status, session);
	}
}

/* Test the Seq class
void SeqClassTest()
{
Seq ss;
ss.shutter(1 * us, 1);
std::cout << "size of the vector" << ss.size() << std::endl;
std::cout << "" << (ss.vector())[0].size() << std::endl;
Sleep(1000);
}
*/

//endregion "FPGA combined sequences"
#pragma endregion

#pragma region "FPGA individual sequences"

//Linearly scan the galvo while the RS is on to acquire a 2D image
U32QV Acquire2D()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	//linear ramp for the galvo
	double Vmax = 1.5;
	double step = 8 * us;


	U32Q linearRampQueue; //Create a queue for the ramps
	U32Q linearRampSegment0 = linearRamp(step, 25 * ms, -Vmax, Vmax); //ramp up the galvo from -Vmax to Vmax
	U32Q linearRampSegment1 = linearRamp(step, 5 * ms, Vmax, -Vmax);  //set the galvo back to -Vmax
	PushQ(linearRampQueue, linearRampSegment0);
	PushQ(linearRampQueue, linearRampSegment1);

	//AO0 = AO1. TRIGGERED BY CONN1/DIO16
	QV[ABUF0] = linearRampQueue;

	//DO0
	QV[DBUF0].push(DigitalOut(4 * us, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));


	return QV;
}

//Pixel clock sequence. The pixel clock starts when the line clock ticks, followed by a wait time 't'
//At 160MHz, the clock increment is 6.25ns = 0.00625us
U32Q PixelClockSeq()
{
	U32Q Q;	//Create a queue

	//INITIAL WAIT TIME. Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us, then the wait time is (62.5-50)/2 = 6.25us
	double t = 6.25*us;
	U16 latency = 2; //latency of detecting the line clock. Calibrate the latency on the oscilloscope
	Q.push(u32pack(us2tick(t) - latency, 0x0000));

	//PIXEL CLOCK TICKS. Everytime HIGH is pushed, the pixel clock "ticks" (flips its state)
	for (U16 ii = 0; ii < Width_pix + 1; ii++) // Npixels+1 because there is one more pixel-clock tick than number of pixels
		Q.push(PixelClock(0.125 * us, 1));
	//Q.push(PixelClock(0.0625 * us, 1));
	return Q; //this returns a queue and not a vector of queues
}

//endregion "FPGA individual sequences"
#pragma endregion


