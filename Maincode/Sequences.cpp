#include "Sequences.h"

int Sequence1(FPGAapi &fpga)
{
	//Create a real-time sequence
	RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 25.2 * ms, galvo1Amp_volt, -galvo1Amp_volt);		//Linear ramp for the galvo
	sequence.pushLinearRamp(GALVO1, galvoTimeStep_us, 1 * ms, -galvo1Amp_volt, galvo1Amp_volt);			//set the output back to the initial value
	//sequence.pushSingleValue(GALVO1, singleAnalogOut(4 * us, galvo1Amp_volt));								//set the output back to the initial value
	fpga.sendRTtoFPGA();
	
	if (!fpga.mStatus)
	{
		PhotonCounter counter(fpga);		//Create a photon counter
		counter.readCount();				//Execute the RT sequence and read the photon count
	}
	return 0;
}