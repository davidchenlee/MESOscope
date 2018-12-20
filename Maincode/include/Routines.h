#pragma once
#include "Devices.h"
#include "Sequencer.h"
//#include <concrt.h> 	//Concurrency::wait(2000);

void mainVision(const FPGAns::FPGA &fpga);
void mainFidelity(const FPGAns::FPGA &fpga);
void testStageTrigAcq(const FPGAns::FPGA &fpga);
void testGalvo(const FPGAns::FPGA &fpga);
void testPixelclock(const FPGAns::FPGA &fpga);
void testAODO(const FPGAns::FPGA &fpga);
void testAOramp(const FPGAns::FPGA &fpga);
void checkDigitalTiming(const FPGAns::FPGA &fpga);
void calibDigitalLatency(const FPGAns::FPGA &fpga);
void calibAnalogLatency(const FPGAns::FPGA &fpga);
void testFilterwheel();
void testShutter(const FPGAns::FPGA &fpga);
void testStagePosition();
void testStageConfig();
void testPMT16X();
void testLaser(const FPGAns::FPGA &fpga);
void testVirtualLaser(const FPGAns::FPGA &fpga);
void testPockels(const FPGAns::FPGA &fpga);
void testRS(const FPGAns::FPGA &fpga);
void testConvertI16toVolt();
void testTiffU8();
void testEthernetSpeed();
void testVibratome(const FPGAns::FPGA &fpga);
void testSequencer();