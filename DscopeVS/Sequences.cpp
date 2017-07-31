#include "Sequences.h"

U32QV Seq1()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//AO1
	QV[0].push(AnalogOut(4 * us, 10));
	QV[0].push(AnalogOut(100 * us, 0));
	QV[0].push(AnalogOut(4 * us, 5));
	QV[0].push(AnalogOut(4 * us, 0));//go back to zero

	//QV[0] = GalvoSeq();

	//AO2
	QV[1].push(AnalogOut(4*us, 0));
	QV[1].push(AnalogOut(100*us, 5));
	QV[1].push(AnalogOut(4*us, 0));
	QV[1].push(AnalogOut(4*us, 0));

	//DO1
	QV[2].push(DigitalOut(4 * us, 1));
	QV[2].push(DigitalOut(100 * us, 0));
	QV[2].push(DigitalOut(4 * us, 1));
	QV[2].push(DigitalOut(4 * us, 0));

	
	//Detector
	QV[3].push(GateDelay(3.125*us));
	for (int ii = 0; ii < 2; ii++)
	{
		QV[3].push(DigitalOut(0.0625 * us, 1));
		QV[3].push(DigitalOut(0.0625 * us, 0));
	}



	return QV;
}

U32QV Seq2()
{
	U32QV QV(Nchan);

	//AO1
	QV[0].push(AnalogOut(4 * us, 10));
	QV[0].push(AnalogOut(4 * us, 0));
	//AO2
	QV[1].push(AnalogOut(4 * us, 10));
	QV[1].push(AnalogOut(4 * us, 0));
	//DO1
	QV[2].push(DigitalOut(4 * us, 1));
	QV[2].push(DigitalOut(4 * us, 0));

	return QV;
}

//this is a queue, not a vector of queues
U32Q GalvoSeq()
{
	double Vmax = 5;
	double step = 4 * ms;
	U32Q Q;
	//linear output
	U32Q linearRamp1 = linearRamp(step, 1 * s, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 1 * s, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 1 * s, Vmax, 0);
	PushQ(Q, linearRamp1);
	PushQ(Q, linearRamp2);
	PushQ(Q, linearRamp3);
	return Q;
}

U32QV GalvoTest()
{
	U32QV QV(Nchan);

	QV[2].push(DigitalOut(62.5 * us, 1));
	QV[2].push(DigitalOut(4 * us, 0));

	return QV;
}


U32QV AnalogLatencyCalib()
{
	//Calibrate DO first, then use it as a time reference
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double delay = 400 * us;
	double step = 4 * us;

	//AO1
	QV[0].push(AnalogOut(step, 10));//initial pulse
	QV[0].push(AnalogOut(step, 0));
	QV[0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[0].push(AnalogOut(step, 5));//final pulse
	QV[0].push(AnalogOut(step, 0));

	//DO1
	QV[2].push(DigitalOut(step, 1));
	QV[2].push(DigitalOut(step, 0));
	QV[2].push(DigitalOut(delay, 0));
	QV[2].push(DigitalOut(step, 1));
	QV[2].push(DigitalOut(step, 0));

	return QV;
}

U32QV DigitalLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);
	double delay = 400 * us;
	double step = 4 * us;

	//AO1
	QV[0].push(AnalogOut(step, 10));//initial pulse
	QV[0].push(AnalogOut(step, 0));
	QV[0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[0].push(AnalogOut(step, 5));//final pulse
	QV[0].push(AnalogOut(step, 0));

	//DO1
	QV[2].push(DigitalOut(step, 1));

	for (int ii = 0; ii < 99; ii++)
		QV[2].push(DigitalOut(step, 0));

	QV[2].push(DigitalOut(step, 1));
	QV[2].push(DigitalOut(step, 0));

	return QV;
}