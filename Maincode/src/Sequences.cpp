#include "Sequences.h"

void seq_main(const FPGAapi &fpga)
{							
	const int wavelength_nm = 940;
	double laserPower_mW = 100 * mW;
	const double FFOVslow_um = 200 * um;	//Full FOV in the slow axis (galvo)
	const double galvo1Vmax_volt = FFOVslow_um * galvo_voltPerUm;
	const double galvoTimeStep_us = 8 * us;

	std::string filename = ".\\Output\\photoncouny";

	PockelsCell pockels(fpga, Pockels1, wavelength_nm);			//Create a pockels cell

	//pockels.manualOn(0);	//For debugging

	Stage stage;


	double newPosition = 18.400;
	stage.moveStage(zz, newPosition);
	stage.printPositionXYZ();
	Sleep(1000);


	//Create a realtime sequence
	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.5 * ms, galvo1Vmax_volt, -galvo1Vmax_volt);		//Linear ramp for the galvo
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Vmax_volt, galvo1Vmax_volt);			//set the output back to the initial value

	//NON-REALTIME SEQUENCE
	for (int ii = 0; ii < 100; ii++)
	{


		Image image(fpga);
		sequence.uploadRT(); //Upload the realtime sequence to the FPGA but don't execute it yet
		image.acquire(filename + " z = " + std::to_string(newPosition)); //Execute the realtime sequence and acquire the image
		
		
		newPosition += 0.001;
		stage.moveStage(zz, newPosition);
		stage.printPositionXYZ();
		laserPower_mW +=0.5;
		pockels.setOutput_mW(laserPower_mW);
		Sleep(1000);
		
	}

	//pockels.setOutputToZero();	//I don't need to do this because normaly the PC is triggered by the scanner

	Logger datalog(filename);
	datalog.record("Wavelength (nm) = ", wavelength_nm);
	datalog.record("Laser power (mW) = ", laserPower_mW);
	datalog.record("FFOV (um) = ", FFOVslow_um);
	datalog.record("Galvo Vmax (V) = ", galvo1Vmax_volt);
	datalog.record("Galvo time step (us) = ", galvoTimeStep_us);
}

Logger::Logger(const std::string filename)
{
	mFileHandle.open(filename + ".txt");
};

Logger::~Logger()
{
	mFileHandle.close();
};

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << std::endl;
}



//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAapi &fpga)
{
	RTsequence sequence(fpga);

	//DO
	sequence.pushSingleValue(DOdebug, packSingleDigital(4 * us, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(4 * us, 0));
	sequence.pushSingleValue(DOdebug, packSingleDigital(4 * us, 0));
	sequence.pushSingleValue(DOdebug, packSingleDigital(4 * us, 0));

	//AO
	sequence.pushSingleValue(GALVO1, packSingleAnalog(4 * us, 5));
	sequence.pushSingleValue(GALVO1, packSingleAnalog(4 * us, 0));
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
	sequence.pushSingleValue(DOdebug, packSingleDigital(pulsewidth, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(4 * us, 0));
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAapi &fpga)
{
	double step = 400 * us;

	RTsequence sequence(fpga);
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 0));
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAapi &fpga)
{
	double step = 4 * us;

	RTsequence sequence(fpga);

	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 1));

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		sequence.pushSingleValue(DOdebug, packSingleDigital(step, 0));

	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 0));
}

//First calibrate the digital channels, then use it as a time reference
void seq_calibAnalogLatency(const FPGAapi &fpga)
{
	double delay = 400 * us;
	double step = 4 * us;

	RTsequence sequence(fpga);
	sequence.pushSingleValue(GALVO1, packSingleAnalog(step, 10));	//Initial pulse
	sequence.pushSingleValue(GALVO1, packSingleAnalog(step, 0));
	sequence.pushLinearRamp(GALVO1, 4 * us, delay, 0, 5*V);			//Linear ramp to accumulate the error
	sequence.pushSingleValue(GALVO1, packSingleAnalog(step, 10));	//Initial pulse
	sequence.pushSingleValue(GALVO1, packSingleAnalog(step, 0));	//Final pulse

	//DO0
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 0));
	sequence.pushSingleValue(DOdebug, packSingleDigital(delay, 0));
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 1));
	sequence.pushSingleValue(DOdebug, packSingleDigital(step, 0));
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