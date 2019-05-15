#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
namespace PMT1XRoutines
{
	void frameByFrameScan(const FPGAns::FPGA &fpga);
	void frameByFrameScanTiling(const FPGAns::FPGA &fpga, const int nSlice);
	void liveScan(const FPGAns::FPGA &fpga);
	void continuousScan(const FPGAns::FPGA &fpga);
	void sequencer(const FPGAns::FPGA &fpga);
}

namespace PMT16XRoutines
{
	void PMT16XframeByFrameScan(const FPGAns::FPGA &fpga);
	void frameByFrameScanTiling(const FPGAns::FPGA &fpga, const int nSlice);
	void liveScan(const FPGAns::FPGA &fpga);
	void continuousScan(const FPGAns::FPGA &fpga);
}

//TESTS
namespace TestRoutines
{
	//FPGA timing
	void digitalLatency(const FPGAns::FPGA &fpga);
	void analogLatency(const FPGAns::FPGA &fpga);
	void pixelclock(const FPGAns::FPGA &fpga);
	void digitalTiming(const FPGAns::FPGA &fpga);
	void analogAndDigitalOut(const FPGAns::FPGA &fpga);
	void analogRamp(const FPGAns::FPGA &fpga);

	//Scanners
	void fineTuneScanGalvo(const FPGAns::FPGA &fpga);
	void resonantScanner(const FPGAns::FPGA &fpga);
	void galvosSync(const FPGAns::FPGA &fpga);

	//Stages
	void stagePosition();
	void stageConfig();

	//Lasers
	void shutter(const FPGAns::FPGA &fpga);
	void pockels(const FPGAns::FPGA &fpga);
	void pockelsRamp(const FPGAns::FPGA &fpga);
	void lasers(const FPGAns::FPGA &fpga);
	void virtualLasers(const FPGAns::FPGA &fpga);

	//Data management
	void convertI16toVolt();
	void tiffU8();
	void ethernetSpeed();
	void multithread();

	//Sequencer
	void sequencerConcurrentTest();
	void locationSequencer();

	//PMT16X
	void PMT16Xconfig();
	void PMT16Xdemultiplex(const FPGAns::FPGA &fpga);
	void PMT16XgavosSyncAndLaser(const FPGAns::FPGA &fpga);

	//Others
	void vibratome(const FPGAns::FPGA &fpga);
	void filterwheel();
	void photobleach(const FPGAns::FPGA &fpga);
	void generateLocationsForBigStitcher();
	int2 nTileToArrayIndices(const int nTile);
}
