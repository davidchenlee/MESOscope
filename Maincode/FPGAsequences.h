#pragma once
#include "FPGAlowlevel.h"
#include <fstream>      //file management
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()


//Individual sequences
void CountPhotons(NiFpga_Status* status, NiFpga_Session session);
U32QV Acquire2D();
U32QV TestAODOSeq();
U32Q PixelClockSeq();
U32Q GalvoLinearRamp();
U32QV GalvoTest();
U32QV DigitalTimingCheck();
U32QV AnalogLatencyCalib();
U32QV DigitalLatencyCalib();
U32Q linearRamp(double dt, double T, double Vi, double Vf);

//Vibratome functions
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel);
int StartVT(NiFpga_Status* status, NiFpga_Session session);

//Combined sequences
void FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session);

