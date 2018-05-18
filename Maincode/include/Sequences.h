#pragma once
#include "Devices.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void seq_main(const FPGAapi::Session &fpga);
void seq_testAOramp(const FPGAapi::Session &fpga);
void seq_checkDigitalTiming(const FPGAapi::Session &fpga);
void seq_calibDigitalLatency(const FPGAapi::Session &fpga);
void seq_calibAnalogLatency(const FPGAapi::Session &fpga);
void seq_testFilterwheel(const FPGAapi::Session &fpga);
void seq_testStages(const FPGAapi::Session &fpga);
