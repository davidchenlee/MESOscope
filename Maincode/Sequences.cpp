#include "Sequences.h"

#pragma region "Combined sequences"
int runCombinedSequence(NiFpga_Status* status, NiFpga_Session session)
{
	/*
	const double RSamplitude_um = 200 * um;
	const double RSamplitude_volt = RSamplitude_um * RS_voltPerUm;
	resonantScanner_SetOutputVoltager(status, session, RSamplitude_volt);
	Sleep(1000);
	resonantScanner_StartStop(status, session, 1);
	Sleep(3000);
	resonantScanner_StartStop(status, session, 0);
	*/

	//To make sure the the filterwheel 1 is set to the correct position
	//FilterWheel();
	//std::cout << endl;
	//Sleep(1000); //wait for the filterwheel to settle

	//Initialize the FPGA
	initializeFPGAvariables(session);

	//Send the commands to the FPGA buffer
	sendCommandsToFPGAbuffer(session, command2DScan());

	//Send the commands to each channel buffers, but do not execute them yet
	executeFPGACommands(session);		

	//Execute the commands and read the photon count
	readPhotonCount(status, session);

	//sendCommand(&status, session, 3 * s, VibratomeBack);
	//setState(&status, session);

	//SECOND ROUND
	if (0)
	{
		//sendCommandsToFPGAbuffer(status, session, TestAODO());
		executeFPGACommands(session);
		triggerFPGAstartImaging(session);
	}

	Sleep(100);
	triggerFIFOflush(session);

	return 0;
}
//endregion "FPGA combined sequences"
#pragma endregion


#pragma region "Individual sequences"

//Scan a frame (image plane) by linearly scan the galvo while the RS is on
VQU32 command2DScan()
{
	VQU32 vectorOfQueues(Nchan);			//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	
	PixelClock pixelClock;					//Create an empty pixel clock
	//vectorOfQueues[PCLOCK] = pixelClock.PixelClock::PixelClockEqualDuration();
	vectorOfQueues[PCLOCK] = pixelClock.PixelClock::PixelClockEqualDistance();

	//Linear ramp for the galvo
	const double galvoAmplitude_um = 200 * um;
	const double galvoAmplitude_volt = galvoAmplitude_um * Galvo_voltPerUm;
	//const double galvoAmplitude_volt = 2.5;
	const double galvoTimeStep_us = 8 * us;

	QU32 linearRampSegment0 = generateLinearRamp(galvoTimeStep_us, 25 * ms, galvoAmplitude_volt, -galvoAmplitude_volt);	//Ramp up the galvo from -galvoAmplitude_volt to galvoAmplitude_volt
	
	vectorOfQueues[IDgalvo1] = linearRampSegment0;
	vectorOfQueues[IDgalvo1].push(singleAnalogOut(4 * us, galvoAmplitude_volt));								//Set the galvo back to -galvoAmplitude_volt
	
	/*//debugger
	vectorOfQueues[IDgalvo1].push(generateSingleAnalogOut(4 * us, 0));
	vectorOfQueues[IDgalvo1].push(generateSingleAnalogOut(4 * us, 5));
	vectorOfQueues[IDgalvo1].push(generateSingleAnalogOut(4 * us, 0));
	vectorOfQueues[IDgalvo1].push(generateSingleAnalogOut(4 * us, 5));
	vectorOfQueues[IDgalvo1].push(generateSingleAnalogOut(4 * us, 0));
	*/
	
	//DO0
	vectorOfQueues[IDshutter1].push(singleDigitalOut(4 * us, 1));
	vectorOfQueues[IDshutter1].push(singleDigitalOut(4 * us, 0));
	vectorOfQueues[IDshutter1].push(singleDigitalOut(4 * us, 0));
	vectorOfQueues[IDshutter1].push(singleDigitalOut(4 * us, 0));

	return vectorOfQueues;
}

//endregion "Individual sequences"
#pragma endregion

VQU32 RTsequence::mVectorOfQueues(Nchan);	//initialize the static member

RTsequence eee;