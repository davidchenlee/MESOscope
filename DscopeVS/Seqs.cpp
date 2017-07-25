#include "Seqs.h"

U32QV Seq1()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	U32QV QV(Nchannels);
	for (U32 ii = 0; ii < Nchannels; ++ii)
		QV[ii] = {};

	//AO1
	QV[0].push(AnalogOut(4 * _us, 10));
	QV[0].push(AnalogOut(4 * _us, 0));
	//QV[0].push(AnalogOut(1*_ms, 10));
	//QV[0].push(AnalogOut(1*_us, 0);
	QV[0] = NewConcatenatedQ(QV[0], linearRamp(4 * _us, 1 * _ms, 0, 5));
	//QV[0].push(AnalogOut(1*_ms, 0));
	QV[0].push(AnalogOut(4 * _us, 5));
	QV[0].push(AnalogOut(4 * _us, 0));//go back to zero

	QV[0] = GalvoSeq();

	//AO2
	//QV[1].push(u32pack(us2tick(5*_us), VO0));
	//QV[1].push(u32pack(4*_ms, VO1));
	//QV[1].push(u32pack(4*_us, VO2));
	//QV[1].push(u32pack(4*_us, VO3));

	//DO1
	QV[2].push(DigitalOut(4 * _us, 1));
	QV[2].push(DigitalOut(4 * _us, 0));
	QV[2].push(DigitalOut(1 * _ms, 0));
	QV[2].push(DigitalOut(4 * _us, 1));
	QV[2].push(DigitalOut(4 * _us, 0));

	return QV;
}

U32QV Seq2()
{
	U32QV QV(Nchannels);

	//AO1
	QV[0].push(AnalogOut(4 * _us, 10));
	QV[0].push(AnalogOut(4 * _us, 0));
	//AO2
	QV[1].push(AnalogOut(4 * _us, 10));
	QV[1].push(AnalogOut(4 * _us, 0));
	//DO1
	QV[2].push(DigitalOut(4 * _us, 1));
	QV[2].push(DigitalOut(4 * _us, 0));

	return QV;
}

U32Q GalvoSeq()
{
	double Vmax = 5;
	double step = 4 * _ms;
	//linear output
	U32Q linearRamp1 = linearRamp(step, 1 * _s, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 1 * _s, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 1 * _s, Vmax, 0);
	return NewConcatenatedQ(NewConcatenatedQ(linearRamp1, linearRamp2), linearRamp3);//overwrites FIFO[0] with a linear ramp
}

