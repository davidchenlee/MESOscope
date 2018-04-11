#include "FPGAsequences.h"

#pragma region "FPGA combined sequences"

int FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session)
{

	/*
	const double RSamplitude_um = 200 * um;
	const double RSamplitude_volt = RSamplitude_um * RS_voltPerUm;
	ResonantScanner_SetOutputVoltager(status, session, RSamplitude_volt);
	Sleep(1000);
	ResonantScanner_StartStop(status, session, 1);
	Sleep(3000);
	ResonantScanner_StartStop(status, session, 0);
	*/


	//Send the commands to the FPGA
	SendOutQueue(status, session, Scan2D());

	//Trigger the data acquisition
	TriggerFIFOIN(status, session);		

	//Read the photon count
	ReadPhotonCount(status, session);






	//SECOND ROUND
	if (0)
	{
		//SendOutQueue(status, session, TestAODO());
		TriggerFIFOIN(status, session);
		TriggerLineGate(status, session);
	}

	return 0;
}
//endregion "FPGA combined sequences"
#pragma endregion

#pragma region "FPGA individual sequences"

//Acquire a 2D image by linearly scan the galvo while the RS is on
U32QV Scan2D()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	//linear ramp for the galvo
	const double GalvoAmplitude_um = 200 * um;
	const double GalvoAmplitude_volt = GalvoAmplitude_um * Galvo_voltPerUm;
	const double GalvoStep = 8 * us;


	U32Q linearRampQueue; //Create a queue for the ramps
	U32Q linearRampSegment0 = linearRamp(GalvoStep, 25 * ms, -GalvoAmplitude_volt, GalvoAmplitude_volt); //ramp up the galvo from -GalvoAmplitude_volt to GalvoAmplitude_volt
	U32Q linearRampSegment1 = linearRamp(GalvoStep, 5 * ms, GalvoAmplitude_volt, -GalvoAmplitude_volt);  //set the galvo back to -GalvoAmplitude_volt
	PushQ(linearRampQueue, linearRampSegment0);
	//PushQ(linearRampQueue, linearRampSegment1);

	//AO0 = AO1. TRIGGERED BY THE LINE CLOCK
	QV[ABUF0] = linearRampQueue;
	QV[ABUF0].push(AnalogOut(4 * us, -GalvoAmplitude_volt));

	//DO0
	QV[DBUF0].push(DigitalOut(4 * us, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));

	return QV;
}

//Pixel clock sequence. The pixel clock starts when the line clock ticks, followed by a waiting time 't'
//At 160MHz, the clock increment is 6.25ns = 0.00625us
U32Q PixelClockSeq()
{
	U32Q Q;	//Create a queue

	//INITIAL WAITING TIME. Currently, there are 400 pixels and the dwell time is 125ns. Then, 400*125ns = 50us. A line-scan lasts 62.5us, then the waiting time is (62.5-50)/2 = 6.25us
	const double t = 6.25*us;
	U16 latency = 2; //latency of detecting the line clock. Calibrate the latency on the oscilloscope
	Q.push(u32pack(us2tick(t) - latency, 0x0000));

	//PIXEL CLOCK TICKS. Everytime HIGH is pushed, the pixel clock "ticks" (flips its state)
	for (U16 ii = 0; ii < Width_pixPerFrame + 1; ii++) // Npixels+1 because there is one more pixel-clock tick than number of pixels
		Q.push(PixelClock(0.125 * us, 1));
	//Q.push(PixelClock(0.0625 * us, 1));
	return Q; //this returns a queue and not a vector of queues
}

//endregion "FPGA individual sequences"
#pragma endregion


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