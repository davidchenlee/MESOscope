#pragma once
#include "Devices.h"

//Combined sequences
int runCombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
VQU32 command2DScan();