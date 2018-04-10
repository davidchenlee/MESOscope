#pragma once
#include "Word.h"
#include "windows.h"	//for  Sleep
//using namespace Const;

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
void MainSequence(NiFpga_Status* status, NiFpga_Session session);


//Vibratome functions
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel);
int StartVT(NiFpga_Status* status, NiFpga_Session session);


//FPGA functions
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);