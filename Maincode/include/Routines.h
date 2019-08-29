#pragma once
#include "Devices.h"
#include "Sequencer.h"

//MAIN SEQUENCES
namespace Routines
{
	void stepwiseScan(FPGA &fpga);
	void contScanZ(FPGA &fpga);
	void contScanX(FPGA &fpga);
	void sequencer(FPGA &fpga, const bool run);
	void liveScan(FPGA &fpga);
	//void frameByFrameZscanTilingXY(const FPGA &fpga, const int nSlice);
}

//TESTS
namespace TestRoutines
{
	//FPGA timing
	void digitalLatency(FPGA &fpga);
	void analogLatency(FPGA &fpga);
	void pixelclock(FPGA &fpga);
	void digitalTiming(FPGA &fpga);
	void analogAndDigitalOut(FPGA &fpga);
	void analogRamp(FPGA &fpga);

	//Scanners
	void fineTuneScanGalvo(FPGA &fpga);
	void resonantScanner(FPGA &fpga);
	void galvosSync(FPGA &fpga);
	void gavosLaserSync(FPGA &fpga);

	//Stages
	void stagePosition();
	void stageConfig();

	//Lasers
	void shutter(FPGA &fpga);
	void pockels(FPGA &fpga);
	void pockelsRamp(FPGA &fpga);
	void lasers(FPGA &fpga);
	void virtualLasers(FPGA &fpga);
	void photobleach(FPGA &fpga);

	//Data management
	void convertI16toVolt();
	void ethernetSpeed();
	void multithread();
	void clipU8();

	//Postprocessing
	void correctImage();
	void quickStitcher();
	void locateSample();
	void isDark();
	void vectorOfObjects();

	//Sequence
	void sequencerConcurrentTest();
	//void locationSequence();
	void generateLocationsForBigStitcher();
	INDICES2 nTileToArrayIndices(const int nTile);

	//PMT16X
	void PMT16Xconfig();
	void PMT16Xdemultiplex(const FPGA &fpga);

	//Others
	void vibratome(const FPGA &fpga);
	void filterwheel();
	void collectorLens();
	void openCV();
}
