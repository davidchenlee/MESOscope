#include "Sequences.h"

void Sequence1(FPGAapi &fpga) //Pass fpga by reference to be able to modify fpga.mVectorOfQueue
{
	//Create a realtime sequence
	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.5 * ms, galvo1Amp_volt, -galvo1Amp_volt);		//Linear ramp for the galvo
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Amp_volt, galvo1Amp_volt);			//set the output back to the initial value
	fpga.sendRTtoFPGA();
	
	PhotonCounter counter(fpga);		//Create a photon counter
	counter.readCount();				//Execute the RT sequence and read the photon count

}