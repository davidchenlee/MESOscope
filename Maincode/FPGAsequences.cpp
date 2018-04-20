#include "FPGAsequences.h"

#pragma region "Combined sequences"

int combinedSequence(NiFpga_Status* status, NiFpga_Session session)
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

	//Initialize the FPGA
	initializeFPGA(status, session);

	//Send the commands to the FPGA buffer
	sendCommandsToFPGAbuffer(status, session, Scan2D());

	//Send the commands to the FPGA sub-buffers for each channel, but do not execute yet
	triggerFPGAdistributeCommandsAmongChannels(status, session);		

	//Execute the commands and read the photon count
	readPhotonCount(status, session);


	//SECOND ROUND
	if (0)
	{
		//sendCommandsToFPGAbuffer(status, session, TestAODO());
		triggerFPGAdistributeCommandsAmongChannels(status, session);
		triggerFPGAstartImaging(status, session);
	}

	return 0;
}
//endregion "FPGA combined sequences"
#pragma endregion

#pragma region "Individual sequences"

//Scan a frame (image plane) by linearly scan the galvo while the RS is on
U32QV Scan2D()
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


int initializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteU8(session, NiFpga_FPGAvi_ControlU8_PhotonCounterInputSelector, PhotonCounterInput));			//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
	NiFpga_MergeStatus(status, NiFpga_WriteU8(session, NiFpga_FPGAvi_ControlU8_LineClockInputSelector, LineClockInput));					//Select the Line clock: resonant scanner or function generator
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));										//control-sequence trigger
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));									//data-acquisition trigger

	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout_tick));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16)SyncDOtoAO_tick));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_SyncAODOtoLineGate, (U16)SyncAODOtoLineGate_tick));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(NlinesAllFrames + NFrames * NlinesSkip)));			//Total number of lines in all the frames, including the skipped lines
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)HeightPerFrame_pix));								//Number of lines in a frame, without including the skipped lines
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(HeightPerFrame_pix + NlinesSkip)));		//Number of lines in a frame including the skipped lines

																																						//Shutters
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

	//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Resonant scanner
	NiFpga_MergeStatus(status, NiFpga_WriteI16(session, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));											//Output voltage
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));											//Turn on/off

																																			//Debugging
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));										//FIFO OUT

	//Initialize all the channels with zero. No need if NiFpga_Finalize() is at the end of the main code
	
	/*
	U32QV vectorOfQueues(Nchan);
	for (int chan = 0; chan < Nchan; chan++)
	{
		vectorOfQueues[chan].push(0);
	}	
	sendCommandsToFPGAbuffer(status, session, vectorOfQueues);
	triggerFPGAdistributeCommandsAmongChannels(status, session);
	triggerFPGAstartImaging(status, session);
	Sleep(100);
	*/
	

	std::cout << "FPGA initialize-variables status: " << *status << std::endl;

	return 0;
}