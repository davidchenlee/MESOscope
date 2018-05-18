#include "Sequences.h"

void seq_main(const FPGAapi::Session &fpga)
{		

	const int wavelength_nm = 940;
	const double laserPower_mW = 100 * mW;
	const double FFOV_slowAxis_um = 200 * um;	//Galvo full FOV in the slow axis

	const std::string filename = "photoncount";


	double newPosition = 18.400;
	/*
	Stage stage;
	stage.moveStage(zz, newPosition);
	stage.printPositionXYZ();
	Sleep(1000);
	*/
		
	//Create a realtime sequence
	FPGAapi::RTsequence sequence(fpga);

	//Create a galvo RT sequence
	Galvo galvo(sequence, GALVO1);
	const double duration_ms = 25.5 * ms;
	const double galvoTimeStep_us = 8 * us;
	const double posMAX = FFOV_slowAxis_um / 2;
	galvo.positionLinearRamp(galvoTimeStep_us, duration_ms, posMAX, -posMAX);		//Linear ramp for the galvo
	galvo.positionLinearRamp(galvoTimeStep_us, 1 * ms, -posMAX, posMAX);			//set the output back to the initial value

	//Create a pockels cell RT sequence
	PockelsCell pockels(sequence, POCKELS1, wavelength_nm);
	pockels.powerLinearRamp(400 * us, duration_ms, laserPower_mW, laserPower_mW);
	pockels.outputToZero();

	const int nFrames = 1;
	//NON-REALTIME SEQUENCE
	for (int ii = 0; ii < nFrames; ii++)
	{
		sequence.uploadRT(); //Upload the realtime sequence to the FPGA but don't execute it yet
		
		Image image(fpga);
		image.acquire(filename + " z = " + toString(newPosition,4)); //Execute the realtime sequence and acquire the image
		
		/*
		newPosition += 0.001;
		stage.moveStage(zz, newPosition);
		stage.printPositionXYZ();
		laserPower_mW += 0.5;
		pockels.setOutput_mW(laserPower_mW);
		Sleep(1000);
		*/
	}

	//pockels.setOutputToZero();	//I don't need to do this because normaly the PC is triggered by the scanner

	Logger datalog(filename);
	datalog.record("Wavelength (nm) = ", wavelength_nm);
	datalog.record("Laser power (mW) = ", laserPower_mW);
	datalog.record("FFOV (um) = ", FFOV_slowAxis_um);
	datalog.record("Galvo Vmax (V) = ", galvo1Vmax_V);
	datalog.record("Galvo time step (us) = ", galvoTimeStep_us);
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAapi::Session &fpga)
{
	FPGAapi::RTsequence sequence(fpga);

	//DO
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);

	//AO
	sequence.pushAnalogSinglet(GALVO1, 4 * us, 5);
	sequence.pushAnalogSinglet(GALVO1, 4 * us, 0);
}

void seq_testAOramp(const FPGAapi::Session &fpga)
{
	double Vmax = 5;
	double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, 0, -Vmax);
	sequence.pushLinearRamp(GALVO1, step, 20 * ms, -Vmax, Vmax);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, Vmax, 0);

	double pulsewidth = 300 * us;
	sequence.pushDigitalSinglet(DOdebug, pulsewidth, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAapi::Session &fpga)
{
	double step = 400 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAapi::Session &fpga)
{
	double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);

	sequence.pushDigitalSinglet(DOdebug, step, 1);

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		sequence.pushDigitalSinglet(DOdebug, step, 0);

	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

//First calibrate the digital channels, then use it as a time reference
void seq_calibAnalogLatency(const FPGAapi::Session &fpga)
{
	double delay = 400 * us;
	double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushAnalogSinglet(GALVO1, step, 10);	//Initial pulse
	sequence.pushAnalogSinglet(GALVO1, step, 0);
	sequence.pushLinearRamp(GALVO1, 4 * us, delay, 0, 5*V);			//Linear ramp to accumulate the error
	sequence.pushAnalogSinglet(GALVO1, step, 10);	//Initial pulse
	sequence.pushAnalogSinglet(GALVO1, step, 0);	//Final pulse

	//DO0
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
	sequence.pushDigitalSinglet(DOdebug, delay, 0);
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

void seq_testFilterwheel(const FPGAapi::Session &fpga)
{
	//Filterwheel FW(FW1);
	//FW.setFilterPosition(BlueLight);

	//Laser laser;
	//laser.setWavelength();
	//std::cout << laser.readWavelength();
}

void seq_testStages(const FPGAapi::Session &fpga)
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