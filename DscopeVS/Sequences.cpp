#include "Sequences.h"

U32QV Seq1()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//AO0
	QV[AO0].push(AnalogOut(4 * us, 5));
	QV[AO0].push(AnalogOut(4 * us, 0));
	QV[AO0].push(AnalogOut(4 * us, 5));
	QV[AO0].push(AnalogOut(4 * us, 0));//go back to zero

	//DO0
	QV[DO0].push(DigitalOut(4 * us, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));
	QV[DO0].push(DigitalOut(4 * us, 0));
	QV[DO0].push(DigitalOut(4 * us, 0));

	//QV[AO0] = GalvoSeq();

	//AO1
	QV[AO1].push(AnalogOut(4*us, 5));
	QV[AO1].push(AnalogOut(4*us, 0));
	QV[AO1].push(AnalogOut(4*us, 5));
	QV[AO1].push(AnalogOut(4*us, 0));

	//DO1
	QV[DO1].push(DigitalOut(4 * us, 1));
	QV[DO1].push(DigitalOut(4 * us, 0));
	QV[DO1].push(DigitalOut(4 * us, 0));
	QV[DO1].push(DigitalOut(4 * us, 0));

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	return QV;
}


U32QV Seq2()
{
	U32QV QV(Nchan);

	//AO0
	QV[AO0].push(AnalogOut(4 * us, 10));
	QV[AO0].push(AnalogOut(4 * us, 0));
	//AO1
	QV[AO1].push(AnalogOut(4 * us, 10));
	QV[AO1].push(AnalogOut(4 * us, 0));
	//DO0
	QV[DO0].push(DigitalOut(4 * us, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));

	return QV;
}



//Pixel clock sequence. The pixel clock starts when the line clock ticks, followed by a wait time 't'
//Currently the clock increment is 6.25ns = 0.00625us
U32Q PixelClockSeq()
{
	U32Q Q;	
	
	//INITIAL WAIT TIME
	double t = 3.125*us;
	U16 latency = 2; //latency of detecting the line clock. Calibrate the latency on the oscilloscope
	Q.push(u32pack(us2tick(t) - latency, 0x0000));

	//PIXEL CLOCK TICKS. Everytime HIGH is pushed, the pixel clock "ticks" (flips its state)
	for (U32 ii = 0; ii < Npixels + 1; ii++) // Npixels+1 because there is one more pixel-clock ticks than number of pixels
		Q.push(PixelClock(0.0625 * us, 1));
	return Q; //this returns a queue and not a vector of queues
}



U32Q GalvoSeq()
{
	double Vmax = 0.05;
	double step = 100 * us;
	U32Q Q;
	//linear output
	U32Q linearRamp1 = linearRamp(step, 5 * ms, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 10 * ms, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 5 * ms, Vmax, 0);
	PushQ(Q, linearRamp1);
	PushQ(Q, linearRamp2);
	PushQ(Q, linearRamp3);
	return Q; //this returns a queue and not a vector of queues
}

U32QV GalvoTest()
{
	U32QV QV(Nchan);
	//QV[AO0] = GalvoSeq();
	double pulsewidth = 300 * us;

	QV[AO0].push(AnalogOut(4 * us, 0.000));
	QV[AO0].push(AnalogOut(pulsewidth, -0.020));
	QV[AO0].push(AnalogOut(4 * us, 0.000));

	QV[DO0].push(DigitalOut(pulsewidth, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));
	return QV;
}


U32QV AnalogLatencyCalib()
{
	//Calibrate DO first, then use it as a time reference
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double delay = 400 * us;
	double step = 4 * us;

	//AO0
	QV[AO0].push(AnalogOut(step, 10));//initial pulse
	QV[AO0].push(AnalogOut(step, 0));
	QV[AO0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[AO0].push(AnalogOut(step, 5));//final pulse
	QV[AO0].push(AnalogOut(step, 0));

	//DO0
	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));
	QV[DO0].push(DigitalOut(delay, 0));
	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));

	return QV;
}

U32QV DigitalLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double step = 4 * us;

	//DO0
	QV[DO0].push(DigitalOut(step, 1));

	for (U32 ii = 0; ii < 99; ii++)
		QV[DO0].push(DigitalOut(step, 0));

	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));

	return QV;
}