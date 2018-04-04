#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
using namespace Const;

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
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);

//FPGA initialization and trigger
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);