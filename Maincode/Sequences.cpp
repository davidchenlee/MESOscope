#include "Sequences.h"

int Sequence1(FPGAapi fpga)
{
	//Linear ramp for the galvo
	const double galvoAmplitude_um = 200 * um;
	const double galvoAmplitude_volt = galvoAmplitude_um * Galvo_voltPerUm;
	const double galvoTimeStep_us = 8 * us;

	//Create a real-time sequence
	RTsequence sequence(&fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25 * ms, galvoAmplitude_volt, -galvoAmplitude_volt);
	sequence.pushSingleValue(GALVO1, singleAnalogOut(4 * us, galvoAmplitude_volt));


	fpga.sendRTtoFPGA();

	//Create a photon counter
	PhotonCounter counter(fpga);

	//Execute the RT sequence and read the photon count
	counter.readCount();

	return 0;
}