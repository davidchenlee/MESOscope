#pragma once
#include "Devices.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void seq_main(const FPGAns::FPGA &fpga);
void seq_contAcquisition(const FPGAns::FPGA &fpga);
void seq_testPixelclock(const FPGAns::FPGA &fpga);
void seq_testAODO(const FPGAns::FPGA &fpga);
void seq_testAOramp(const FPGAns::FPGA &fpga);
void seq_checkDigitalTiming(const FPGAns::FPGA &fpga);
void seq_calibDigitalLatency(const FPGAns::FPGA &fpga);
void seq_calibAnalogLatency(const FPGAns::FPGA &fpga);
void seq_testFilterwheel();
void seq_testStageSetPosition();
void seq_testStageTriggerConfig();
void seq_testmPMT();
void seq_testPockels(const FPGAns::FPGA &fpga);
void seq_testLaserComm(const FPGAns::FPGA &fpga);
void seq_testRS(const FPGAns::FPGA &fpga);
void seq_testConvertI16toVolt();
void seq_testGalvoSync(const FPGAns::FPGA &fpga);
void seq_testTiff();
