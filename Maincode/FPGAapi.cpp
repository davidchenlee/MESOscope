#include "FPGAapi.h"

FPGAapi::FPGAapi(): mVectorOfQueues(Nchan)
{
	mStatus = NiFpga_Initialize();		//Must be called before any other FPGA calls
	std::cout << "FPGA initialize status: " << mStatus << std::endl;

	if (NiFpga_IsNotError(mStatus))		//Check for any FPGA error
	{
		mStatus = NiFpga_Open(Bitfile, NiFpga_FPGAvi_Signature, "RIO0", 0, &mSession);		//Opens a session, downloads the bitstream
																							//1=no run, 0=run
		std::cout << "FPGA open-session status: " << mStatus << std::endl;
	}
}

FPGAapi::~FPGAapi()
{
};

void FPGAapi::close()
{
	if (NiFpga_IsNotError(mStatus))
	{
		mStatus = NiFpga_Close(mSession, 1);			//Closes the session to the FPGA. The FPGA resets (Re-downloads the FPGA bitstream to the target, the outputs go to zero)
														//unless either another session is still open or you use the NiFpga_CloseAttribute_NoResetIfLastSession attribute.
														//0 resets, 1 does not reset
		std::cout << "FPGA closing-session status: " << mStatus << std::endl;
	}

	//You must call this function after all other function calls if NiFpga_Initialize succeeds. This function unloads the NiFpga library.
	mStatus = NiFpga_Finalize();
	std::cout << "FPGA finalize status: " << mStatus << std::endl;
}

int FPGAapi::initialize()
{
	//Initialize the FPGA variables. See 'Const.cpp' for the definition of each variable
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_PhotonCounterInputSelector, PhotonCounterInput));			//Debugger. Use the PMT-pulse simulator as the input of the photon-counter
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU8(mSession, NiFpga_FPGAvi_ControlU8_LineClockInputSelector, LineClockInput));					//Select the Line clock: resonant scanner or function generator
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));									//control-sequence trigger
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));									//data-acquisition trigger

	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_FIFOtimeout, (U16)FIFOtimeout_tick));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_Nchannels, (U16)Nchan));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncDOtoAO, (U16)SyncDOtoAO_tick));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_SyncAODOtoLineGate, (U16)SyncAODOtoLineGate_tick));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesAll, (U16)(NlinesAllFrames + NFrames * NlinesSkip)));			//Total number of lines in all the frames, including the skipped lines
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFrame, (U16)HeightPerFrame_pix));							//Number of lines in a frame, without including the skipped lines
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteU16(mSession, NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips, (U16)(HeightPerFrame_pix + NlinesSkip)));	//Number of lines in a frame including the skipped lines

																																						//Shutters
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter1, 0));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_Shutter2, 0));

	//Vibratome control
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_start, 0));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_back, 0));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_forward, 0));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_VT_NC, 0));

	//Resonant scanner
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteI16(mSession, NiFpga_FPGAvi_ControlI16_RS_voltage, 0));										//Output voltage
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_RS_ON_OFF, 0));										//Turn on/off

																																			//Debugging
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteArrayBool(mSession, NiFpga_FPGAvi_ControlArrayBool_Pulsesequence, pulseArray, Npulses));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOOUTdebug, 0));									//FIFO OUT

	std::cout << "FPGA initialization status: " << mStatus << std::endl;

	return 0;
}


//Send every single queue in VectorOfQueue to the FPGA bufer
//For this, concatenate all the single queues in a single long queue. THE QUEUE POSITION DETERMINES THE TARGETED CHANNEL	
//Then transfer the elements in the long queue to an array to interface the FPGA
//Alternatively, the single queues could be transferred directly to the array, but why bothering...
int FPGAapi::sendCommandsToFPGAbuffer()
{
	QU32 allQueues;								//Create a single long queue
	for (int i = 0; i < Nchan; i++)
	{
		allQueues.push(mVectorOfQueues[i].size());			//Push the number of elements in each individual queue VectorOfQueues[i]
		while (!mVectorOfQueues[i].empty())
		{
			allQueues.push(mVectorOfQueues[i].front());		//Push all the elemets in individual queue VectorOfQueues[i] to allQueues
			mVectorOfQueues[i].pop();
		}
	}


	const int sizeFIFOqueue = allQueues.size();		//Total number of elements in all the queues 

	if (sizeFIFOqueue > FIFOINmax)
		std::cerr << "WARNING in " << __FUNCTION__ << ": FIFO IN overflow" << std::endl;

	U32* FIFO = new U32[sizeFIFOqueue];				//Create an array for interfacing the FPGA	
	for (int i = 0; i < sizeFIFOqueue; i++)
	{
		FIFO[i] = allQueues.front();				//Transfer the queue elements to the array
		allQueues.pop();
	}
	allQueues = {};									//Cleanup the queue (C++11 style)

													//Send the data to the FPGA through the FIFO
	const U32 timeout = -1; // in ms. A value -1 prevents the FIFO from timing out
	U32 r; //empty elements remaining

	NiFpga_Status status = NiFpga_WriteFifoU32(mSession, NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN, FIFO, sizeFIFOqueue, timeout, &r);

	std::cout << "FPGA FIFO status: " << status << std::endl;
	delete[] FIFO;//cleanup the array

	return 0;
}

void FPGAapi::sendRTsequenceToFPGA()
{
	this->sendCommandsToFPGAbuffer();

	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 1));
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FIFOINtrigger, 0));
	std::cout << "Pulse trigger status: " << mStatus << std::endl;
}

//Execute the commands
int FPGAapi::triggerFPGAstartImaging()
{
	NiFpga_Status status = NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 1);
	NiFpga_MergeStatus(&status, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_LineGateTrigger, 0));
	std::cout << "Acquisition trigger status: " << status << std::endl;

	return 0;
}


//Trigger the FIFO flushing
int FPGAapi::triggerFIFOflush()
{
	mStatus = NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 1);
	NiFpga_MergeStatus(&mStatus, NiFpga_WriteBool(mSession, NiFpga_FPGAvi_ControlBool_FlushTrigger, 0));
	std::cout << "Flush trigger status: " << mStatus << std::endl;

	return 0;
}


void FPGAapi::printFPGAstatus(char functionName[])
{
	if (mStatus < 0)
		std::cerr << "ERROR: '" << functionName << "' exited with FPGA code: " << mStatus << std::endl;
	if (mStatus > 0)
		std::cerr << "WARNING: '" << functionName << "' exited with FPGA code: " << mStatus << std::endl;
}


/*
int i;
for(i=0; i<size; i=i+1){
data[i] = i*5000;


for(i=0; i<size; i=i+1){
printf("%i\n",data[i]);
}
getchar();
*/

/*
int16_t val = -32769;
char hex[16];
sprintf(hex, "%x", ((val + (1 << 16)) % (1 << 16)) );
puts(hex);
getchar();*/


/*the AO reads a I16, specifically
0x7FFF = 32767
0xFFFF = -1
0x8000 = -32768*/

/*
printf("%i\n", VOUT(10));
getchar();*/