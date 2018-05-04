#pragma once
#include "Devices.h"

extern const double FFOVslow_um;
extern const double galvo1Amp_volt;
extern const double galvoTimeStep_us;

void Sequence1(const FPGAapi &fpga);

void testAOramp(const FPGAapi &fpga);
void checkDigitalTiming(const FPGAapi &fpga);
void calibDigitalLatency(const FPGAapi &fpga);
void calibAnalogLatency(const FPGAapi &fpga);