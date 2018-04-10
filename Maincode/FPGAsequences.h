#pragma once
#include "FPGAlowlevel.h"

//Combined sequences
void FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV Acquire2D();
U32Q PixelClockSeq();



