#pragma once
#include "NiFpga_FPGAvi.h"
#include <iostream>
#include <fstream>      //file management
#include "Const.h"
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()
using namespace Const;


//Low-level functions

/*Define the full path of the bitfile*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

void printHex(U32 input);
U32 u32pack(U16 t, U16 x);
U16 us2tick(double x);
I16 volt2I16(double x);
U32 AnalogOut(double t, double V);
U32 DigitalOut(double t, bool DO);
U32 PixelClock(double t, bool DO);
U32Q PushQ(U32Q& headQ, U32Q& tailQ);
U32Q linearRamp(double dt, double T, double Vi, double Vf);
void CountPhotons(NiFpga_Status* status, NiFpga_Session session);


//Individual sequences
U32QV Acquire2D();
U32QV TestAODOSeq();
U32Q PixelClockSeq();
U32Q GalvoLinearRamp();
U32QV GalvoTest();
U32QV DigitalTimingCheck();
U32QV AnalogLatencyCalib();
U32QV DigitalLatencyCalib();

//Combined sequences
void FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session);


//Vibratome functions
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel);
int StartVT(NiFpga_Status* status, NiFpga_Session session);


//FPGA functions
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);

