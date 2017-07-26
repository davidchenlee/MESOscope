#include "Sequences.h"

U32QV Seq1()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchan);

	//AO1
	QV[0].push(AnalogOut(4 * us, 10));
	QV[0].push(AnalogOut(4 * us, 0));
	//QV[0].push(AnalogOut(1*ms, 10));
	//QV[0].push(AnalogOut(1*us, 0);
	QV[0] = PushQ(QV[0], linearRamp(4 * us, 1 * ms, 0, 5));
	//QV[0].push(AnalogOut(1*_ms, 0));
	QV[0].push(AnalogOut(4 * us, 5));
	QV[0].push(AnalogOut(4 * us, 0));//go back to zero

	QV[0] = GalvoSeq();

	//AO2
	QV[1].push(AnalogOut(4*us, 0));
	QV[1].push(AnalogOut(1*ms, 5));
	QV[1].push(AnalogOut(4*us, 0));
	QV[1].push(AnalogOut(4*us, 0));

	//DO1
	QV[2].push(DigitalOut(4 * us, 1));
	QV[2].push(DigitalOut(4 * us, 0));
	QV[2].push(DigitalOut(1 * ms, 0));
	QV[2].push(DigitalOut(4 * us, 1));
	QV[2].push(DigitalOut(4 * us, 0));

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

