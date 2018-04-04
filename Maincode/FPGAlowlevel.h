#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include <fstream>      //file management
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()
using namespace Const;

/*Define the full path of the bitfile*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

//Low-level FPGA functions
void printHex(U32 input);
U32 u32pack(U16 t, U16 x);
U16 us2tick(double x);
I16 volt2I16(double x);
U32 AnalogOut(double t, double V);
U32 DigitalOut(double t, bool DO);
U32 PixelClock(double t, bool DO);
U32Q PushQ(U32Q& headQ, U32Q& tailQ);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
U32Q linearRamp(double dt, double T, double Vi, double Vf);
void CountPhotons(NiFpga_Status* status, NiFpga_Session session);


//FPGA initialization and trigger
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void TriggerFIFOIN(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);

//Combined sequences
void FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session);

