#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
void discreteScanZ(const FPGAns::FPGA &fpga);
void continuousScanZ(const FPGAns::FPGA &fpga);

//CALIBRATION
void calibDigitalLatency(const FPGAns::FPGA &fpga);
void calibAnalogLatency(const FPGAns::FPGA &fpga);
void fineTuneGalvoScan(const FPGAns::FPGA &fpga);

//TESTS
void testGalvo(const FPGAns::FPGA &fpga);
void testPixelclock(const FPGAns::FPGA &fpga);
void testAODO(const FPGAns::FPGA &fpga);
void testAOramp(const FPGAns::FPGA &fpga);
void testDigitalTiming(const FPGAns::FPGA &fpga);
void testFilterwheel();
void testShutter(const FPGAns::FPGA &fpga);
void testStagePosition();
void testStageConfig();
void testPMT16X();
void testLasers(const FPGAns::FPGA &fpga);
void testVirtualLasers(const FPGAns::FPGA &fpga);
void testPockels(const FPGAns::FPGA &fpga);
void testRS(const FPGAns::FPGA &fpga);
void testConvertI16toVolt();
void testTiffU8();
void testEthernetSpeed();
void testVibratome(const FPGAns::FPGA &fpga);
void testSequencer();
void testThread();