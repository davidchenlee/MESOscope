#pragma once
#include "Devices.h"

//Combined sequences
int runCombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV generate2DScan();
int initializeFPGA(NiFpga_Status* status, NiFpga_Session session);
U32Q generateLinearRamp(double dt, double T, double Vi, double Vf);



