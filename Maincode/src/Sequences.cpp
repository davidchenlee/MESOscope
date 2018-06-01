#include "Sequences.h"

/*
There are basically 2 imaging modes :
1. Frame by frame: For each frame, a RT sequence is created, loaded onto the fpga, and a corresponding image is acquired. The z stage is moved after each image is acquired.
2. Continuous: A single long RT sequence contains all the frames. Such sequence is loaded onto the fpga and run once. A stream of images is acquired. The z stage moves continuously
*/

void seq_main(const FPGAapi::Session &fpga)
{	
	const int wavelength_nm = 1040;
	double laserPower_mW = 100 * mW;
	const double FFOVgalvo_um = 200 * um;	//Galvo full FOV in the slow axis

	const std::string filename = "PHAL";

	/*
	Stage stage;
	const double3 newPosition_mm = { 37.570, 12.700, 18.440 };
	stage.moveStage3(newPosition_mm);
	stage.waitForMovementToStop3();
	stage.printPosition3();
	double3 position_mm = stage.readPosition3_mm();
	*/
	
	const double duration_ms = 25 * ms; //62.5us * 400 pixels
	const double galvoTimeStep_us = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	const int nFrames = 1;
	//NON-REALTIME SEQUENCE
	for (int ii = 0; ii < nFrames; ii++)
	{
		//Create a realtime sequence
		FPGAapi::RTsequence sequence(fpga);

		//Create a galvo RT sequence
		Galvo galvo(sequence, GALVO1);
		galvo.positionLinearRamp(galvoTimeStep_us, duration_ms, posMax_um, -posMax_um);		//Linear ramp for the galvo
		//galvo.positionLinearRamp(galvoTimeStep_us, 1 * ms, -posMax_um, posMax_um);		//set the output back to the initial value

		//Create a pockels cell RT sequence
		PockelsCell pockels(sequence, POCKELS1, wavelength_nm);
		//pockels.powerLinearRamp(galvoTimeStep_us, duration_ms, laserPower_mW, laserPower_mW);
		//pockels.voltageToZero();
		pockels.voltageLinearRamp(galvoTimeStep_us, duration_ms, 1*V, 1*V);
		//pockels.scalingLinearRamp(1.0, 1.0);
	
		sequence.uploadRT(); //Upload the realtime sequence to the FPGA but don't execute it yet
		
		Image image(fpga);
		//image.acquire(filename + " x = " + toString(position_mm.at(xx), 3) + " y = " + toString(position_mm.at(yy), 3) + " z = " + toString(position_mm.at(zz),3), 1); //Execute the realtime sequence and acquire the image
		image.acquire(filename); //Execute the realtime sequence and acquire the image
		
		/*
		newPosition_mm += 0.001;
		stage.moveStage(zz, newPosition_mm);
		stage.printPosition3();
		laserPower_mW += 0.5;
		*/
		Sleep(1000);
		
		
	}

	Logger datalog(filename);
	datalog.record("Wavelength (nm) = ", wavelength_nm);
	datalog.record("Laser power (mW) = ", laserPower_mW);
	datalog.record("FFOV (um) = ", FFOVgalvo_um);
	datalog.record("Galvo Max position (um) = ", posMax_um);
	datalog.record("Galvo time step (us) = ", galvoTimeStep_us);
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAapi::Session &fpga)
{
	FPGAapi::RTsequence sequence(fpga);

	//DO
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);

	//AO
	sequence.pushAnalogSinglet(GALVO1, 8 * us, 4);
	sequence.pushAnalogSinglet(GALVO1, 4 * us, 0);

	sequence.uploadRT();	//Upload the realtime sequence to the FPGA but don't execute it yet
	sequence.triggerRT();	//Execute the realtime sequence
}

void seq_testAOramp(const FPGAapi::Session &fpga)
{
	const double Vmax = 5;
	const double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, 0, -Vmax);
	sequence.pushLinearRamp(GALVO1, step, 20 * ms, -Vmax, Vmax);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, Vmax, 0);

	const double pulsewidth = 300 * us;
	sequence.pushDigitalSinglet(DOdebug, pulsewidth, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAapi::Session &fpga)
{
	const double step = 400 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAapi::Session &fpga)
{
	const double step = 4 * us;

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
	const double delay = 400 * us;
	const double step = 4 * us;

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
	//const double newPosition_mm = 19.000;
	Stage stage;
	//stage.printPosition3();

	//stage.moveStage(zz, newPosition_mm);
	//stage.waitForMovementToStop(zz);
	//stage.printPosition3();
	
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