#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
namespace MainRoutines
{
	void frameByFrameScan(const FPGAns::FPGA &fpga);
	void frameByFrameScan_LocationList(const FPGAns::FPGA &fpga, const int nSlice);
	void liveScan(const FPGAns::FPGA &fpga);
	void continuousScan(const FPGAns::FPGA &fpga);
	void sequencer(const FPGAns::FPGA &fpga);
}

//TESTS
namespace TestRoutines
{
	void demultiplexing(const FPGAns::FPGA &fpga);
	void digitalLatency(const FPGAns::FPGA &fpga);
	void analogLatency(const FPGAns::FPGA &fpga);
	void pockels(const FPGAns::FPGA &fpga);
	void pockelsRamp(const FPGAns::FPGA &fpga);
	void photobleach(const FPGAns::FPGA &fpga);
	void galvosSyncFullFrame(const FPGAns::FPGA &fpga);
	void galvosSyncPartialFrame(const FPGAns::FPGA &fpga);
	void fineTuneGalvoScan(const FPGAns::FPGA &fpga);
	void pixelclock(const FPGAns::FPGA &fpga);
	void analogAndDigitalOut(const FPGAns::FPGA &fpga);
	void analogRamp(const FPGAns::FPGA &fpga);
	void digitalTiming(const FPGAns::FPGA &fpga);
	void filterwheel();
	void shutter(const FPGAns::FPGA &fpga);
	void stagePosition();
	void stageConfig();
	void PMT16Xconfig();
	void lasers(const FPGAns::FPGA &fpga);
	void virtualLasers(const FPGAns::FPGA &fpga);
	void resonantScanner(const FPGAns::FPGA &fpga);
	void convertI16toVolt();
	void tiffU8();
	void ethernetSpeed();
	void vibratome(const FPGAns::FPGA &fpga);
	void multithread();
	void sequencerConcurrentTest();
	void locationSequencer();
}
