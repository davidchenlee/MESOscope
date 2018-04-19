#pragma once
#include "FPGAlowlevel.h"

//Combined sequences
int combinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV Scan2D();
int initializeFPGA(NiFpga_Status* status, NiFpga_Session session);




