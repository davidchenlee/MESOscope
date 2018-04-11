#pragma once
#include "FPGAlowlevel.h"

//Combined sequences
int FPGAcombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV Scan2D();
U32Q PixelClockSeq();



