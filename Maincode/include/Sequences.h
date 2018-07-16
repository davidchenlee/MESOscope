#pragma once
#include "Devices.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void seq_main(const FPGAapi::Session &fpga);
void seq_testPixelclock(const FPGAapi::Session &fpga);
void seq_testAODO(const FPGAapi::Session &fpga);
void seq_testAOramp(const FPGAapi::Session &fpga);
void seq_checkDigitalTiming(const FPGAapi::Session &fpga);
void seq_calibDigitalLatency(const FPGAapi::Session &fpga);
void seq_calibAnalogLatency(const FPGAapi::Session &fpga);
void seq_testFilterwheel();
void seq_testStageSetPosition();
void seq_testStageTriggerConfig();
void seq_testmPMT();
void seq_testPockels(const FPGAapi::Session &fpga);
void seq_testLaserComm(const FPGAapi::Session &fpga);