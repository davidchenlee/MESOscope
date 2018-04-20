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
	initializeFPGA(status, session);

	//Send the commands to the FPGA buffer
	sendCommandsToFPGAbuffer(status, session, command2DScan());

	//Send the commands to the FPGA sub-buffers for each channel, but do not execute yet
	triggerFPGAdistributeCommandsAmongChannels(status, session);		

	//Execute the commands and read the photon count
	readPhotonCount(status, session);

	//vibratome_SendCommand(&status, session, 3 * s, VibratomeBack);
	//vibratome_StartStop(&status, session);

	//SECOND ROUND
	if (0)
	{
		//sendCommandsToFPGAbuffer(status, session, TestAODO());
		triggerFPGAdistributeCommandsAmongChannels(status, session);
		triggerFPGAstartImaging(status, session);
	}

	Sleep(100);
	triggerFIFOflush(status, session);

	return 0;
}
//endregion "FPGA combined sequences"
#pragma endregion


#pragma region "Individual sequences"

//Scan a frame (image plane) by linearly scan the galvo while the RS is on
U32QV command2DScan()
{
	U32QV vectorOfQueues(Nchan);			//Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	
	PixelClock pixelClock;					//Create an empty pixel clock
	//vectorOfQueues[PCLOCK] = pixelClock.PixelClock::PixelClockEqualDuration();
	vectorOfQueues[PCLOCK] = pixelClock.PixelClock::PixelClockEqualDistance();

	//Linear ramp for the galvo
	const double galvoAmplitude_um = 200 * um;
	const double galvoAmplitude_volt = galvoAmplitude_um * Galvo_voltPerUm;
	//const double galvoAmplitude_volt = 2.5;
	const double galvoTimeStep_us = 8 * us;

	U32Q linearRampSegment0 = generateLinearRamp(galvoTimeStep_us,25 * ms, galvoAmplitude_volt, -galvoAmplitude_volt);	//Ramp up the galvo from -galvoAmplitude_volt to galvoAmplitude_volt
	
	vectorOfQueues[ABUF0] = linearRampSegment0;
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, galvoAmplitude_volt));								//Set the galvo back to -galvoAmplitude_volt
	
	/*//debugger
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, 0));
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, 5));
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, 0));
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, 5));
	vectorOfQueues[ABUF0].push(generateSingleAnalogOut(4 * us, 0));
	*/
	
	//DO0
	vectorOfQueues[DBUF0].push(generateSingleDigitalOut(4 * us, 1));
	vectorOfQueues[DBUF0].push(generateSingleDigitalOut(4 * us, 0));
	vectorOfQueues[DBUF0].push(generateSingleDigitalOut(4 * us, 0));
	vectorOfQueues[DBUF0].push(generateSingleDigitalOut(4 * us, 0));

	return vectorOfQueues;
}

U32Q generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal)
{
	U32Q queue;
	const bool debug = 0;

	if (TimeStep < AOdt_us)
	{
		std::cerr << "WARNING in " << __func__ << ": time step too small. Time step set to " << AOdt_us << " us" << std::endl;
		TimeStep = AOdt_us;						//Analog output time increment (in us)
		return {};
	}

	const int nPoints = (int)(RampLength / TimeStep);		//Number of points

	if (nPoints <= 1)
	{
		std::cerr << "ERROR in " << __func__ << ": not enought points for the linear ramp" << std::endl;
		std::cerr << "nPoints: " << nPoints << std::endl;
		return {};
	}
	else
	{
		if (debug)
		{
			std::cout << "nPoints: " << nPoints << std::endl;
			std::cout << "time \tticks \tv" << std::endl;
		}

		for (int ii = 0; ii < nPoints; ii++)
		{
			const double V = Vinitial + (Vfinal - Vinitial)*ii / (nPoints - 1);
			queue.push(generateSingleAnalogOut(TimeStep, V));

			if (debug)
				std::cout << (ii + 1) * TimeStep << "\t" << (ii + 1) * convertUs2tick(TimeStep) << "\t" << V << "\t" << std::endl;
		}

		if (debug)
		{
			getchar();
			return {};
		}


	}
	return queue;
}

//endregion "Individual sequences"
#pragma endregion