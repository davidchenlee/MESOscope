#include "Sentence.h"

U32QV Seq1()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA

	//AO0
	QV[AO0].push(AnalogOut(4 * us, 10));
	QV[AO0].push(AnalogOut(4 * us, 0));
	QV[AO0].push(AnalogOut(4 * us, 10));
	QV[AO0].push(AnalogOut(4 * us, 0));//go back to zero

	//DO0
	QV[DO0].push(DigitalOut(4 * us, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));
	QV[DO0].push(DigitalOut(4 * us, 0));
	QV[DO0].push(DigitalOut(4 * us, 0));

	//QV[AO0] = GalvoSeq();


	//CURRENTLY, AO1 AND DO1 ARE TRIGGERED BY CONN1/DIO16
	//AO1
	QV[AO1].push(AnalogOut(4 * us, 5));
	QV[AO1].push(AnalogOut(4 * us, 0));
	QV[AO1].push(AnalogOut(4 * us, 5));
	QV[AO1].push(AnalogOut(4 * us, 0));

	//DO1
	QV[DO1].push(DigitalOut(4 * us, 1));
	QV[DO1].push(DigitalOut(4 * us, 0));
	QV[DO1].push(DigitalOut(4 * us, 0));
	QV[DO1].push(DigitalOut(4 * us, 0));

	//Pixel clock
	QV[PCLOCK] = PixelClockSeq();

	return QV;
}


U32QV Seq2()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA

	//AO0
	QV[AO0].push(AnalogOut(4 * us, 10));
	QV[AO0].push(AnalogOut(4 * us, 0));
	//AO1
	QV[AO1].push(AnalogOut(4 * us, 10));
	QV[AO1].push(AnalogOut(4 * us, 0));
	//DO0
	QV[DO0].push(DigitalOut(4 * us, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));

	return QV;
}



//Pixel clock sequence. The pixel clock starts when the line clock ticks, followed by a wait time 't'
//Currently the clock increment is 6.25ns = 0.00625us
U32Q PixelClockSeq()
{
	U32Q Q;	//Create a queue

	//INITIAL WAIT TIME
	double t = 3.125*us;
	U16 latency = 2; //latency of detecting the line clock. Calibrate the latency on the oscilloscope
	Q.push(u32pack(us2tick(t) - latency, 0x0000));

	//PIXEL CLOCK TICKS. Everytime HIGH is pushed, the pixel clock "ticks" (flips its state)
	for (U32 ii = 0; ii < Npixels + 1; ii++) // Npixels+1 because there is one more pixel-clock ticks than number of pixels
		Q.push(PixelClock(0.0625 * us, 1));
	return Q; //this returns a queue and not a vector of queues
}



U32Q GalvoSeq()
{
	double Vmax = 0.05;
	double step = 100 * us;
	U32Q Q; //Create a queue

	//linear output
	U32Q linearRamp1 = linearRamp(step, 5 * ms, 0, -Vmax);
	U32Q linearRamp2 = linearRamp(step, 10 * ms, -Vmax, Vmax);
	U32Q linearRamp3 = linearRamp(step, 5 * ms, Vmax, 0);
	PushQ(Q, linearRamp1);
	PushQ(Q, linearRamp2);
	PushQ(Q, linearRamp3);
	return Q; //this returns a queue and not a vector of queues
}


U32QV GalvoTest()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	//QV[AO0] = GalvoSeq();
	double pulsewidth = 300 * us;

	QV[AO0].push(AnalogOut(4 * us, 0.000));
	QV[AO0].push(AnalogOut(pulsewidth, -0.020));
	QV[AO0].push(AnalogOut(4 * us, 0.000));

	QV[DO0].push(DigitalOut(pulsewidth, 1));
	QV[DO0].push(DigitalOut(4 * us, 0));
	return QV;
}


U32QV DigitalOutSeq(bool DO)
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double step = 350 * us;

	//DO0
	QV[DO0].push(DigitalOut(step, DO));
	return QV;
}


U32QV DigitalTimingCheck()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double step = 400 * us;

	//DO0
	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));

	return QV;
}


U32QV DigitalLatencyCalib()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double step = 4 * us;

	//DO0
	QV[DO0].push(DigitalOut(step, 1));

	for (U32 ii = 0; ii < 99; ii++)
		QV[DO0].push(DigitalOut(step, 0));

	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));

	return QV;
}


//Calibrate the digital channels first, then use it as a time reference
U32QV AnalogLatencyCalib()
{
	U32QV QV(Nchan); //Create and initialize a vector of queues. Each queue correspond to a channel on the FPGA
	double delay = 400 * us;
	double step = 4 * us;

	//AO0
	QV[AO0].push(AnalogOut(step, 10));//initial pulse
	QV[AO0].push(AnalogOut(step, 0));
	QV[AO0] = PushQ(QV[0], linearRamp(4 * us, delay, 0, 5));//linear ramp to accumulate the error
	QV[AO0].push(AnalogOut(step, 5));//final pulse
	QV[AO0].push(AnalogOut(step, 0));

	//DO0
	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));
	QV[DO0].push(DigitalOut(delay, 0));
	QV[DO0].push(DigitalOut(step, 1));
	QV[DO0].push(DigitalOut(step, 0));

	return QV;
}


//Start running the vibratome
int StartVT(NiFpga_Status* status, NiFpga_Session session)
{
	int dt = 20; //in ms. It has to be ~ 12 ms or longer to 
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_start, 1));
	Sleep(dt);
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_start, 0));

	return 0;
}

//Non-deterministic digital pulse for the vibratome control. The timing fluctuates approx in 1ms
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel)
{
	U8 selectedChannel;
	int minstep = 10; //in ms

	switch (channel)
	{
	case VTback:
		selectedChannel = NiFpga_FPGA_ControlBool_VT_back;
		break;
	case VTforward:
		selectedChannel = NiFpga_FPGA_ControlBool_VT_forward;
		break;
	default:
		std::cout << "ERROR: Selected VT channel is unavailable\n";
		return -1;
	}


	int delay = 1; //used to roughly calibrate the pulse length
	int dt_ms = (int)dt / ms;

	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 1));

	if (dt_ms >= minstep)
		Sleep(dt_ms - delay);
	else
	{
		Sleep(minstep - delay);
		std::cout << "WARNING: time step too small. Time step set to the min = ~" << minstep << "ms \n";
	}
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, selectedChannel, 0));
	
	return 0;
}



/* FPGA FUNCTIONS**************************************************************************************************************************************************************************************/

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session)
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Trigger, 0));//control-sequence trigger
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_Start_acquisition, 0)); //data-acquisition trigger

	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_FIFO_timeout, FIFOtimeout));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Nchannels, Nchan));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Ncounters, Ncounters));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_DO_to_AO, Sync_DO_to_AO));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Sync_AODO_to_LineGate, Sync_AODO_to_LineGate));
	NiFpga_MergeStatus(status, NiFpga_WriteArrayBool(session, NiFpga_FPGA_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(status, NiFpga_WriteU16(session, NiFpga_FPGA_ControlU16_Nmax_lines, Nmaxlines));

	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_start, 0));//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_back, 0));//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_forward, 0));//Vibratome control
	NiFpga_MergeStatus(status, NiFpga_WriteBool(session, NiFpga_FPGA_ControlBool_VT_NC, 0)); //Vibratome control

	//Initialize all the channels with zero. Not needed if NiFpga_Finalize() is at the end of the main code
	/*
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