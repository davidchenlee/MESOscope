#pragma once
#include "Devices.h"

void Sequence1(const FPGAapi &fpga);

void testAOramp(const FPGAapi &fpga);
void checkDigitalTiming(const FPGAapi &fpga);
void calibDigitalLatency(const FPGAapi &fpga);
void calibAnalogLatency(const FPGAapi &fpga);