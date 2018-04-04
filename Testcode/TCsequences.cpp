#include "TCsequences.h"

U32Q TestLinearRamp()
{
	double Vmax = 5;
	double step = 4 * us;
	U32Q Q; //Create a queue

			//linear output
	U32Q linearRamp1 = linearRamp(step, 1 * ms, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 25 * ms, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 1 * ms, Vmax, 0);
	//PushQ(Q, linearRamp1);
	PushQ(Q, linearRamp2);
	//PushQ(Q, linearRamp3);
	return Q; //this returns a queue and not a vector of queues
}

//Test the analog and digital output
U32QV TestAODO()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//AO0
	QV[ABUF0].push(AnalogOut(4 * us, 10));
	QV[ABUF0].push(AnalogOut(4 * us, 0));
	QV[ABUF0].push(AnalogOut(4 * us, 10));
	QV[ABUF0].push(AnalogOut(4 * us, 0));//go back to zero

										 //DO0
	QV[DBUF0].push(DigitalOut(4 * us, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	QV[DBUF0].push(DigitalOut(4 * us, 0));

	//QV[AO0] = GalvoSeq();


	//CURRENTLY, AO1 AND DO1 ARE TRIGGERED BY CONN1/DIO16
	//AO1
	QV[ABUF1].push(AnalogOut(4 * us, 5));
	QV[ABUF1].push(AnalogOut(4 * us, 0));
	QV[ABUF1].push(AnalogOut(4 * us, 5));
	QV[ABUF1].push(AnalogOut(4 * us, 0));

	//DO1
	QV[DBUF1].push(DigitalOut(4 * us, 1));
	QV[DBUF1].push(DigitalOut(4 * us, 0));
	QV[DBUF1].push(DigitalOut(4 * us, 0));
	QV[DBUF1].push(DigitalOut(4 * us, 0));

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	return QV;
}

U32QV TestAODOplusRamp()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	QV[ABUF0] = TestLinearRamp();

	double pulsewidth = 300 * us;

	/*
	QV[AO0].push(AnalogOut(4 * us, 0.000));
	QV[AO0].push(AnalogOut(pulsewidth, 5));
	QV[AO0].push(AnalogOut(4 * us, 0.000));
	*/

	QV[DBUF0].push(DigitalOut(pulsewidth, 1));
	QV[DBUF0].push(DigitalOut(4 * us, 0));
	return QV;
}

U32QV DigitalTimingCheck()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double step = 400 * us;

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}

U32QV DigitalLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double step = 4 * us;

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));

	for (U32 ii = 0; ii < 99; ii++)
		QV[DBUF0].push(DigitalOut(step, 0));

	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}

//Calibrate the digital channels first, then use it as a time reference
U32QV AnalogLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double delay = 400 * us;
	double step = 4 * us;

	//AO0
	QV[ABUF0].push(AnalogOut(step, 10));//initial pulse
	QV[ABUF0].push(AnalogOut(step, 0));
	QV[ABUF0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[ABUF0].push(AnalogOut(step, 5));//final pulse
	QV[ABUF0].push(AnalogOut(step, 0));

	//DO0
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));
	QV[DBUF0].push(DigitalOut(delay, 0));
	QV[DBUF0].push(DigitalOut(step, 1));
	QV[DBUF0].push(DigitalOut(step, 0));

	return QV;
}