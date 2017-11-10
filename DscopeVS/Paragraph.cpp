#include "Paragraph.h"


//Non-deterministic digital pulse. The timing fluctuates in around 0.3ms
void NonDetDigitalPulse(NiFpga_Status* status, NiFpga_Session session, double tstep)
{
	int delay = 4; //used to calibrate the pulse
	int dt_ms = (int) tstep / ms;


	SendOutQueue(status, session, DigitalOutSeq(1));
	TriggerAODO(status, session);

	if (dt_ms >= delay)
		Sleep(dt_ms - delay);
	else
		std::cout << "WARNING: time step too small. Time step set to the min\n";

	SendOutQueue(status, session, DigitalOutSeq(0));
	TriggerAODO(status, session);
}


/* FPGA FUNCTIONS**************************************************************************************************************************************************************************************/

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_FIFO_timeout, FIFOtimeout));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Nchannels, Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Ncounters, Ncounters));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_DO_to_AO, Sync_DO_to_AO));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_AODO_to_LineGate, Sync_AODO_to_LineGate));
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGA_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Nmax_lines, Nmaxlines));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0)); //start acquiring data

	/*
	//Initialize all the channels with zero. Not needed if NiFpga_Finalize() is at the end of the main code
	U32QV QV(Nchan);
	for (U8 ii = 0; ii < Nchan; ii++)
		QV[ii].push(0);
	SendOutQueue(status, session, QV);
	PulseTrigger(status, session);
	*/

	std::cout << "FPGA initialize-variables status: " << *status << "\n";
}



void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& QV)
{
	//take a vector of queues and return it as a single queue
	U32Q allQs;
	for (U32 i = 0; i < Nchan; i++)
	{
		allQs.push(QV[i].size()); //push the number of elements in the queue
		while (!QV[i].empty())
		{
			allQs.push(QV[i].front());
			QV[i].pop();
		}
	}
	//transfer the queue to an array to be sent to the FPGA. THE ORDER DETERMINES THE TARGETED CHANNEL
	U32 sizeFIFOqueue = allQs.size();
	U32* FIFO = new U32[sizeFIFOqueue];
	for (U32 i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQs.front();
		allQs.pop();
	}
	allQs = {};//cleanup the queue in C++11

			   //send the data to the FPGA through the FIFO
	U32 timeout = -1; // in ms. A value -1 prevents the FIFO from timing out
	U32 r; //empty elements remaining

	NiFpga_MergeStatus(status, NiFpga_WriteFifoU32(session, NiFpga_FPGA_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r));

	std::cout << "FPGA FIFO status: " << *status << "\n";
	delete[] FIFO;//cleanup
}

//Main trigger. Trigger FIFO-in, which subsequently triggers AO and DO
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));
	std::cout << "Pulse trigger status: " << *status << "\n";
}

//Trigger the pixel clock, and therefore, counters, and FIFO-out
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session)
{
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 1));
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0));
	std::cout << "Acquisition trigger status: " << *status << "\n";
}