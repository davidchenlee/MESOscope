#pragma once
#include "Devices.h"

//Combined sequences
int runCombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV command2DScan();

class FPGAClassTest
{
	NiFpga_Status status;
	NiFpga_Session session;

public:
	FPGAClassTest();
	~FPGAClassTest();
};
