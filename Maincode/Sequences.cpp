#include "Sequences.h"

void Sequence1(FPGAapi fpga)
{
	//Create a realtime sequence
	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.5 * ms, galvo1Amp_volt, -galvo1Amp_volt);		//Linear ramp for the galvo
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Amp_volt, galvo1Amp_volt);			//set the output back to the initial value
	sequence.sendtoFPGA();

	PockelsCell pockels(fpga, Pockels1);			//Create a pockels cell
	PhotonCounter counter(fpga);		//Create a photon counter
	ResonantScanner RS(fpga);			//Create a resonant scanner
	
	//Create a non-realtime sequence
	pockels.turnOn(5 * V);
	counter.readCount();				//Execute the RT sequence and read the photon count
	pockels.turnOff();

	
	if (0)
		RS.turnOn(200 * um);
	else
		RS.turnOff();
}