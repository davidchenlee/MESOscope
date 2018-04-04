#pragma once
#include "FPGAlowlevel.h"

//Individual sequences
U32QV Acquire2D();
U32Q PixelClockSeq();

//Vibratome functions
int PulseVTcontrol(NiFpga_Status* status, NiFpga_Session session, double dt, VTchannel channel);
int StartVT(NiFpga_Status* status, NiFpga_Session session);

