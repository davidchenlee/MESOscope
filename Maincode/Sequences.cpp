#include "Sequences.h"

int Sequence1(FPGAapi fpga)
{
	//Create a real-time sequence
	RTsequence sequence(&fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25 * ms, galvo1Amp_volt, -galvo1Amp_volt);		//Linear ramp for the galvo
	sequence.pushSingleValue(GALVO1, singleAnalogOut(4 * us, galvo1Amp_volt));								//set the output back to the initial value
	fpga.sendRTtoFPGA();

	//Create a photon counter
	PhotonCounter counter(fpga);

	//Execute the RT sequence and read the photon count
	counter.readCount();

	return 0;
}