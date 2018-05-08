#pragma once
#include "Devices.h"

void seq_main(const FPGAapi &fpga);
void seq_testAOramp(const FPGAapi &fpga);
void seq_checkDigitalTiming(const FPGAapi &fpga);
void seq_calibDigitalLatency(const FPGAapi &fpga);
void seq_calibAnalogLatency(const FPGAapi &fpga);
void seq_testFilterwheel(const FPGAapi &fpga);
void seq_testStages(const FPGAapi &fpga);