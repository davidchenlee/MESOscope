#pragma once
#include "Devices.h"

#include <experimental/filesystem> //standard method in C++14 but not C++11

void seq_main(const FPGAapi &fpga);
void seq_testAOramp(const FPGAapi &fpga);
void seq_checkDigitalTiming(const FPGAapi &fpga);
void seq_calibDigitalLatency(const FPGAapi &fpga);
void seq_calibAnalogLatency(const FPGAapi &fpga);
void seq_testFilterwheel(const FPGAapi &fpga);
void seq_testStages(const FPGAapi &fpga);

std::string file_exists(std::string filename);