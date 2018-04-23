#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include <fstream>      //file management
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()
using namespace Const;

/*Define the full path of the bitfile. The bitfile is the FPGA code*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

//Low-level FPGA functions
void printHex(int input);
U32 packU32(U16 t, U16 x);
U16 convertUs2tick(double x);
I16 convertVolt2I16(double x);
U32 singleAnalogOut(double t, double V);
U32 singleDigitalOut(double t, bool DO);
U32 singlePixelClock(double t, bool DO);
QU32 generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal);
void concatenateQueues(QU32& headQ, QU32 tailQ);
int sendCommandsToFPGAbuffer(NiFpga_Session session, VQU32& VectorOfQueues);


//FPGA initialization and trigger
int initializeFPGA(NiFpga_Session session);
int sendFPGAchannelBuffers(NiFpga_Session session);
int triggerFPGAstartImaging(NiFpga_Session session);
int triggerFIFOflush(NiFpga_Session session);
int configureFIFO(NiFpga_Session session, U32 depth);

void printFPGAstatus(NiFpga_Status status, char functionName[]);



class FPGAapi
{
	NiFpga_Status mStatus;
	NiFpga_Session mSession;

public:
	VQU32 mVectorOfQueues;
	FPGAapi();
	~FPGAapi();
	int initialize();
	void loadRTsequenceOnFPGA();
	NiFpga_Session getSession();





};

