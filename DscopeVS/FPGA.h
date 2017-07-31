#pragma once
#include "NiFpga_FPGA.h"
#include <iostream>
#include "Const.h"
using namespace Const;

/*Define the full path of the bitfile*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;

//Prototypes
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
void PulseTrigger(NiFpga_Status* status, NiFpga_Session session);

void printHex(U16 input);
U32 u32pack(U16 t, U16 x);
U16 us2tick(double x);
I16 AOUT(double x);
U32 AnalogOut(double t, double V);
U32 DigitalOut(double t, bool DO);
U32 GateDelay(double t);
U32Q PushQ(U32Q& headQ, U32Q& tailQ);
U32Q linearRamp(double dt, double T, double Vi, double Vf);