#pragma once
#include "Devices.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void seq_main(const FPGAapi::FPGAsession &fpga);
void seq_testAOramp(const FPGAapi::FPGAsession &fpga);
void seq_checkDigitalTiming(const FPGAapi::FPGAsession &fpga);
void seq_calibDigitalLatency(const FPGAapi::FPGAsession &fpga);
void seq_calibAnalogLatency(const FPGAapi::FPGAsession &fpga);
void seq_testFilterwheel(const FPGAapi::FPGAsession &fpga);
void seq_testStages(const FPGAapi::FPGAsession &fpga);
