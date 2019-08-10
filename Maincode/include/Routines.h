#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
namespace PMT16XRoutines
{
	void frameByFrameZscan(const FPGA &fpga);
	void liveScan(const FPGA &fpga);
	void contZscan(const FPGA &fpga);
	void sequencer(const FPGA &fpga);
	//void frameByFrameZscanTilingXY(const FPGA &fpga, const int nSlice);
	void snapshot(const RTcontrol &RTcontrol, const Sequence &sequence, VirtualLaser &virtualLaser, Galvo &rescanner, Stage &stage);
}

//TESTS
namespace TestRoutines
{
	//FPGA timing
	void digitalLatency(const FPGA &fpga);
	void analogLatency(const FPGA &fpga);
	void pixelclock(const FPGA &fpga);
	void digitalTiming(const FPGA &fpga);
	void analogAndDigitalOut(const FPGA &fpga);
	void analogRamp(const FPGA &fpga);

	//Scanners
	void fineTuneScanGalvo(const FPGA &fpga);
	void resonantScanner(const FPGA &fpga);
	void galvosSync(const FPGA &fpga);

	//Stages
	void stagePosition();
	void stageConfig();

	//Lasers
	void shutter(const FPGA &fpga);
	void pockels(const FPGA &fpga);
	void pockelsRamp(const FPGA &fpga);
	void lasers(const FPGA &fpga);
	void virtualLasers(const FPGA &fpga);

	//Data management
	void convertI16toVolt();
	void tiffU8();
	void ethernetSpeed();
	void multithread();
	void clipU8();

	//Sequence
	void sequencerConcurrentTest();
	void locationSequence();

	//PMT16X
	void PMT16Xconfig();
	void PMT16Xdemultiplex(const FPGA &fpga);
	void PMT16XgavosSyncAndLaser(const FPGA &fpga);

	//Others
	void vibratome(const FPGA &fpga);
	void filterwheel();
	void collectorLens();
	void photobleach(const FPGA &fpga);
	void generateLocationsForBigStitcher();
	int2 nTileToArrayIndices(const int nTile);
	void openCV();
}
