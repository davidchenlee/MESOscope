#include "Sequences.h"

void Sequence1(const FPGAapi &fpga)
{

	const int wavelength_nm = 1040;
	const double laserPower_mW = 60 * mW;

	//REALTIME SEQUENCE
	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.5 * ms, galvo1Amp_volt, -galvo1Amp_volt);		//Linear ramp for the galvo
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Amp_volt, galvo1Amp_volt);			//set the output back to the initial value
	sequence.loadRTsequenceonFPGA();

	PockelsCell pockels(fpga, Pockels1, wavelength_nm);			//Create a pockels cell
	
	//NON-REALTIME SEQUENCE
	pockels.turnOn_mW(laserPower_mW);
	//pockels.turnOn_volt(2 * V);
	sequence.runRTsequence();		//Execute the RT sequence and read the photon count
	pockels.turnOff();

	/*
	ResonantScanner RS(fpga);		//Create a resonant scanner
	if (0)
		RS.turnOn_mW(200 * um);
	else
		RS.turnOff();
	*/
	RTsequence sequence2(sequence);
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void TestAODO(const FPGAapi &fpga)
{
	RTsequence sequence(fpga);

	//DO
	sequence.pushSingleValue(DOdebug, singleDigitalOut(4 * us, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(4 * us, 0));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(4 * us, 0));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(4 * us, 0));

	//AO
	sequence.pushSingleValue(GALVO1, singleAnalogOut(4 * us, 5));
	sequence.pushSingleValue(GALVO1, singleAnalogOut(4 * us, 0));
}

void testAOramp(const FPGAapi &fpga)
{
	double Vmax = 5;
	double step = 4 * us;

	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, 0, -Vmax);
	sequence.pushLinearRamp(GALVO1, step, 20 * ms, -Vmax, Vmax);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, Vmax, 0);

	double pulsewidth = 300 * us;
	sequence.pushSingleValue(DOdebug, singleDigitalOut(pulsewidth, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(4 * us, 0));
}

//Generate a long digital pulse and check the duration with the oscilloscope
void checkDigitalTiming(const FPGAapi &fpga)
{
	double step = 400 * us;

	RTsequence sequence(fpga);
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void calibDigitalLatency(const FPGAapi &fpga)
{
	double step = 4 * us;

	RTsequence sequence(fpga);

	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));

	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));
}

//First calibrate the digital channels, then use it as a time reference
void calibAnalogLatency(const FPGAapi &fpga)
{
	double delay = 400 * us;
	double step = 4 * us;

	RTsequence sequence(fpga);
	sequence.pushSingleValue(GALVO1, singleAnalogOut(step, 10));	//initial pulse
	sequence.pushSingleValue(GALVO1, singleAnalogOut(step, 0));
	sequence.pushLinearRamp(GALVO1, 4 * us, delay, 0, 5*V);			//linear ramp to accumulate the error
	sequence.pushSingleValue(GALVO1, singleAnalogOut(step, 10));	//initial pulse
	sequence.pushSingleValue(GALVO1, singleAnalogOut(step, 0));		//final pulse

	//DO0
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(delay, 0));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));
}