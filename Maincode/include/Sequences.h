#pragma once
#include "Devices.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void seq_main(const FPGAns::FPGA &fpga);
void seq_mainFidelity(const FPGAns::FPGA &fpga);
void seq_testGalvo(const FPGAns::FPGA &fpga);
void seq_testPixelclock(const FPGAns::FPGA &fpga);
void seq_testAODO(const FPGAns::FPGA &fpga);
void seq_testAOramp(const FPGAns::FPGA &fpga);
void seq_checkDigitalTiming(const FPGAns::FPGA &fpga);
void seq_calibDigitalLatency(const FPGAns::FPGA &fpga);
void seq_calibAnalogLatency(const FPGAns::FPGA &fpga);
void seq_testFilterwheel();
void seq_testShutter(const FPGAns::FPGA &fpga);
void seq_testStagePosition();
void seq_testStageConfig();
void seq_testPMT16X();
void seq_testLaser(const FPGAns::FPGA &fpga);
void seq_testVirtualLaser(const FPGAns::FPGA &fpga);
void seq_testPockels(const FPGAns::FPGA &fpga);
void seq_testRS(const FPGAns::FPGA &fpga);
void seq_testConvertI16toVolt();
void seq_testTiffU8();
void seq_testEthernetSpeed();
void seq_testVibratome(const FPGAns::FPGA &fpga);
void seq_testStageTrigAcq(const FPGAns::FPGA &fpga);
void seq_scanEntireSample();