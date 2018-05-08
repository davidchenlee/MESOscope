#include "Sequences.h"

void seq_main(const FPGAapi &fpga)
{
	const int wavelength_nm = 940;
	const double laserPower_mW = 60 * mW;
	const double FFOVslow_um = 200 * um;	//Full FOV in the slow axis (galvo)
	const double galvo1Vmax_volt = FFOVslow_um * galvo_voltPerUm;
	const double galvoTimeStep_us = 8 * us;
	

	for (int ii = 0; ii < 1; ii++)
	{
		//REALTIME SEQUENCE
		RTsequence sequence(fpga);
		sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.5 * ms, galvo1Vmax_volt, -galvo1Vmax_volt);		//Linear ramp for the galvo
		sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Vmax_volt, galvo1Vmax_volt);			//set the output back to the initial value
		sequence.uploadRTsequenceToFPGA();

		//NON-REALTIME SEQUENCE
		PockelsCell pockels(fpga, Pockels1, wavelength_nm);			//Create a pockels cell
		pockels.turnOn_mW(laserPower_mW);
		sequence.runRTsequence();									//Execute the RT sequence and read the photon count
		pockels.turnOff(); //warning: saving data delays the calling this function
	}
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAapi &fpga)
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

void seq_testAOramp(const FPGAapi &fpga)
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
void seq_checkDigitalTiming(const FPGAapi &fpga)
{
	double step = 400 * us;

	RTsequence sequence(fpga);
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 1));
	sequence.pushSingleValue(DOdebug, singleDigitalOut(step, 0));
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAapi &fpga)
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
void seq_calibAnalogLatency(const FPGAapi &fpga)
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

void seq_testFilterwheel(const FPGAapi &fpga)
{
	//Filterwheel FW(FW1);
	//FW.setFilterPosition(BlueLight);

	//Laser laser;
	//laser.setWavelength();
	//std::cout << laser.readWavelength();
}

void seq_testStages(const FPGAapi &fpga)
{
	const double newPosition = 18.5520;
	//const double newPosition = 19.000;
	Stage stage;
	//stage.printPositionXYZ();

	//stage.moveStage(zz, newPosition);
	//stage.waitForMovementStop(zz);
	//stage.printPositionXYZ();
	
	int input = 1;
	while (input)
	{
		std::cout << "Stage X position = " << stage.downloadPosition_mm(xx) << std::endl;
		std::cout << "Stage Y position = " << stage.downloadPosition_mm(yy) << std::endl;
		std::cout << "Stage X position = " << stage.downloadPosition_mm(zz) << std::endl;

		std::cout << "Enter command: ";
		std::cin >> input;
		//input = 0;
	}
	getchar();
}