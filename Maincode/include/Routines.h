#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
namespace MainRoutines
{
	void discreteScanZ(const FPGAns::FPGA &fpga);
	void continuousScanZ(const FPGAns::FPGA &fpga);
}

//CALIBRATION
namespace CalibrationRoutines
{
	void digitalLatency(const FPGAns::FPGA &fpga);
	void analogLatency(const FPGAns::FPGA &fpga);
	void fineTuneGalvoScan(const FPGAns::FPGA &fpga);
}

//TESTS
namespace TestRoutines
{
	void galvo(const FPGAns::FPGA &fpga);
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
	void pockels(const FPGAns::FPGA &fpga);
	void pockelsRamp(const FPGAns::FPGA &fpga);
	void resonantScanner(const FPGAns::FPGA &fpga);
	void convertI16toVolt();
	void tiffU8();
	void ethernetSpeed();
	void vibratome(const FPGAns::FPGA &fpga);
	void sequencer();
	void sequencerLight(const FPGAns::FPGA &fpga);
	void multithread();
}
