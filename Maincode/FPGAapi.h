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
U32 generateSingleAnalogOut(double t, double V);
U32 generateSingleDigitalOut(double t, bool DO);
U32 generateSinglePixelClock(double t, bool DO);
U32Q generateLinearRamp(double dt, double T, double Vi, double Vf);
U32Q concatenateQueues(U32Q& headQ, U32Q& tailQ);
int sendCommandsToFPGAbuffer(NiFpga_Status* status, NiFpga_Session session, U32QV& VectorOfQueues);


//FPGA initialization and trigger
int initializeFPGAvariables(NiFpga_Status* status, NiFpga_Session session);
int triggerFPGAdistributeCommands(NiFpga_Status* status, NiFpga_Session session);
int triggerFPGAstartImaging(NiFpga_Status* status, NiFpga_Session session);
int triggerFIFOflush(NiFpga_Status* status, NiFpga_Session session);
int configureFIFO(NiFpga_Status* status, NiFpga_Session session, U32 depth);

