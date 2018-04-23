#include "TCsequences.h"

//Test the analog and digital output and the relative timing wrt the pixel clock
VQU32 TestAODO()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	VQU32 QV(Nchan);

	//AO0
	//QV[ABUF2].push(generateSingleAnalogOut(4 * us, 10));
	//QV[ABUF2].push(generateSingleAnalogOut(4 * us, 0));
	//QV[ABUF2].push(generateSingleAnalogOut(4 * us, 10));
	//QV[ABUF2].push(generateSingleAnalogOut(4 * us, 0));//go back to zero

	//DO0
	QV[IDshutter1].push(singleDigitalOut(4 * us, 1));
	QV[IDshutter1].push(singleDigitalOut(4 * us, 0));
	QV[IDshutter1].push(singleDigitalOut(4 * us, 0));
	QV[IDshutter1].push(singleDigitalOut(4 * us, 0));

	//QV[AO0] = GalvoSeq();


	//CURRENTLY, AO1 AND DO1 ARE TRIGGERED BY THE LINE CLOCK
	//AO0
	QV[IDgalvo1].push(singleAnalogOut(4 * us, 5));
	QV[IDgalvo1].push(singleAnalogOut(4 * us, 0));
	//QV[IDgalvo1].push(generateSingleAnalogOut(4 * us, 5));
	//QV[IDgalvo1].push(generateSingleAnalogOut(4 * us, 0));

	//DO0
	//QV[DBUF1].push(generateSingleDigitalOut(4 * us, 1));
	//QV[DBUF1].push(generateSingleDigitalOut(4 * us, 0));
	//QV[DBUF1].push(generateSingleDigitalOut(4 * us, 0));
	//QV[DBUF1].push(generateSingleDigitalOut(4 * us, 0));

	//Pixel clock
	PixelClock pixelClock;
	QV[PCLOCK] = pixelClock.PixelClock::PixelClockEqualDuration();

	return QV;
}

VQU32 TestAODOandRamp()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	VQU32 QV(Nchan);

	//Generate linear ramps
	double Vmax = 5;
	double step = 4 * us;
	QU32 Q; //Create a queue
	QU32 linearRamp1 = generateLinearRamp(step, 2 * ms, 0, -Vmax);
	QU32 linearRamp2 = generateLinearRamp(step, 20 * ms, -Vmax, Vmax);
	QU32 linearRamp3 = generateLinearRamp(step, 2 * ms, Vmax, 0);
	//concatenateQueues(Q, linearRamp1);
	concatenateQueues(Q, linearRamp2);
	//concatenateQueues(Q, linearRamp3);
	QV[IDgalvo1] = Q;
	Q = {}; //clean up

	double pulsewidth = 300 * us;

	/*
	QV[AO0].push(generateSingleAnalogOut(4 * us, 0.000));
	QV[AO0].push(generateSingleAnalogOut(pulsewidth, 5));
	QV[AO0].push(generateSingleAnalogOut(4 * us, 0.000));
	*/

	QV[IDshutter1].push(singleDigitalOut(pulsewidth, 1));
	QV[IDshutter1].push(singleDigitalOut(4 * us, 0));
	return QV;
}

//Generate a long digital pulse and check the duration with the oscilloscope
VQU32 DigitalTimingCheck()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	VQU32 QV(Nchan);
	double step = 400 * us;

	//DO0
	QV[IDshutter1].push(singleDigitalOut(step, 1));
	QV[IDshutter1].push(singleDigitalOut(step, 0));

	return QV;
}


//Generate many short digital pulses and check the overall duration with the oscilloscope
VQU32 DigitalLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	VQU32 QV(Nchan);
	double step = 4 * us;

	//DO0
	QV[IDshutter1].push(singleDigitalOut(step, 1));

	//many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		QV[IDshutter1].push(singleDigitalOut(step, 0));

	QV[IDshutter1].push(singleDigitalOut(step, 1));
	QV[IDshutter1].push(singleDigitalOut(step, 0));

	return QV;
}

//First, calibrate the digital channels, then use it as a time reference
VQU32 AnalogLatencyCalib()
{
	//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	VQU32 QV(Nchan);
	double delay = 400 * us;
	double step = 4 * us;

	//AO0
	QV[IDgalvo1].push(singleAnalogOut(step, 10));//initial pulse
	QV[IDgalvo1].push(singleAnalogOut(step, 0));
	QV[IDgalvo1] = concatenateQueues(QV[0], generateLinearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[IDgalvo1].push(singleAnalogOut(step, 5));//final pulse
	QV[IDgalvo1].push(singleAnalogOut(step, 0));

	//DO0
	QV[IDshutter1].push(singleDigitalOut(step, 1));
	QV[IDshutter1].push(singleDigitalOut(step, 0));
	QV[IDshutter1].push(singleDigitalOut(delay, 0));
	QV[IDshutter1].push(singleDigitalOut(step, 1));
	QV[IDshutter1].push(singleDigitalOut(step, 0));

	return QV;
}