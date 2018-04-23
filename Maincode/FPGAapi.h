#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
using namespace Const;

/*Define the full path of the bitfile. The bitfile is the FPGA code*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

class FPGAapi
{	
public:
	NiFpga_Status mStatus;
	NiFpga_Session mSession;
	VQU32 mVectorOfQueues;

	FPGAapi();
	~FPGAapi();

	void close();
	int initialize();
	int sendCommandsToFPGAbuffer();
	void sendRTsequenceToFPGA();
	int triggerFPGAstartImaging();
	int triggerFIFOflush();
	void printFPGAstatus(char functionName[]);
};

