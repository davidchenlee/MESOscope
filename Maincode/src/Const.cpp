#include "Const.h"

//Constants that are never changed
namespace Constants
{
	extern const std::string folderPath{ "D:\\_output_local\\" };
	//extern const std::string folderPath{ "Z:\\_output_remote\\" };
	extern const std::string bitfilePath{ "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" };	//Define the full path of the bitfile (compiled LV code that runs on the FPGA)
	extern const std::string openclFilePath{ "D:\\OwnCloud\\Codes\\MESOscope\\Maincode\\src\\" };		//OpenCL kernel code

	//GENERAL CONSTANTS
	extern const double PI{ 3.1415926535897 };
	extern const int us{ 1 };								//Microsecond
	extern const int ms{ 1000 * us };						//Millisecond
	extern const int sec{ 1000 * ms };						//Second
	extern const int um{ 1 };								//Micron	
	extern const int mm{ 1000 * um };						//Millimeter
	extern const double mmps{ 1. * mm / sec };				//Millimeters per second
	extern const int V{ 1 };								//Volt
	extern const int mW{ 1 };								//Milliwatt
	extern const double mWpum{ 1. * mW / um };				//mW per micron

	//LASERS
	extern const double laserPulsePeriod{ 0.0125 * us };	//The pulse repetition rate of VISION and FIDELITY is 80 MHz

	//PIXELCLOCK
	extern const double pixelDwellTime{ 0.1625 * us };		//= 13 * 12.5 ns = 162.5 ns
	extern const double lineclockHalfPeriod{ 63.05 * us };	//Half-period of the resonant scanner. I measure 25.220 ms over 400 half oscillations. Therefore, the average half-period is 25200us/400 = 63.05 us
															//The forward and backward times differ slightly and the difference depends on the scanning amplitude
															//For example, forward = 63.14 us, backwards = 62.99 us, diff = 150 ns (i.e., ~ 1 pixel)
															//The measured RS period (126.1 us) seems to be independent of the scanning amplitude
	//FPGA
	extern const int AOmax{ 10 * V };						//Max voltage of the AOs
	extern const int tickPerUs{ 160 };						//Number of ticks in 1 us. It corresponds to the FPGA's clock
	extern const double usPerTick{ 1. / 160 };				//Time step of the FPGA's clock
	extern const U32 tMIN_tick{ 2 };						//Min ticks allowed = 2 because DO and AO have a latency of 2 ticks
	extern const int AO_tMIN{ 2 * us };						//Time step of the analog output. The AO channels has a delay of >1 us 
	extern const int DOdelay_tick{ 4 * 74 };				//Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
															//WARNING: use the same cable length when calibrating different FPGA outputs. It may need re-calibration
															//because I placed the comparison logics for gating AFTER the line counter instead of before
	extern const int nPreframes{ 4 };						//Number of lineclocks delaying the frameclock (and framegate) wrt the preframeclock (and preframegate)
															//This is for triggering the pockels and rescanner slightly earlier and adjusting the timing by via delay
	extern const double linegateTimeout{ 100 * ms };		//In LV, timeout the start of the data acquisition. Otherwise, Lineclock (from the RS) could false trigger the acquisition
															//e.g., 1. the RS is first off; 2. the control sequence is triggered; 3. the RS is turned on. 4. the acquisition will be triggered
	extern const double postsequenceTimer{ 200 * ms };		//Time after the sequence ends because the motion monitor of the z stage bounces and false-triggers the acq sequence. Enabled only if a stage acts as the main trigger
	extern const double stageDebounceTimer{ 20. * ms};		//Stage motion monitor debouncer
	extern const int FIFOtimeout_tick{ 100 };				//Timeout of the all the FIFOS on the FPGA
	extern const int FIFOINmax{ 32773 };					//Depth of FIFOIN (host-to-target). WARNING: This number MUST match the LV implementation on the FPGA!

	//POCKELS
	extern const double pockelsFirstFrameDelay{ 112. * us };//Delay of the Pockels wrt the preframeclock. The pockels is turned on early to avoid transient overshooting
	extern const double pockelsSecondaryDelay{ 0 };			//Delay of the Pockels wrt the preframeclock in the subsequent frames 

	//GALVOS
	extern const double scanGalvoDelay{ 150 * us };									//Adjust 'scanGalvoDelay' until the bead position in a fordward scan coincides with that of a backward scan
	extern const double rescanGalvoDelay{ 0. * us };								//This does not seem to be very sensitive. Look at the rescanner's ramp on the scope and sync it with the scanner's ramp
	extern const GALVOcalib scannerCalib{ 0.02417210 * V / um , 0.0 * V };			//Calibration factor of the scan galvo. Last calib 31/7/2018 (a larger voltage steers the excitation beam towards the negative dir of the x-stage)

	//Calibration factor to sync the rescanner with the scanner to keep the fluorescence emission fixed at the detector
	//To find both parameters, image beads with a single laser beam at full FOV (i.e. 300x560 pixels) and look at the tiffs in all the PMT channels
	//The beads should show up in the selected channel only
	//Adjust 'mVoltagePerDistance' until all the beads show up in the same selected PMT16X channel
	//Adjust 'mRescanVoltageOffset' to center the beads on the selected PMT16X channel
	//By increasing mVoltagePerDistance, the top beads in a Tiff appear first, then the bottom ones
	//A positive mVoltageOffset steers the beam towards CH00 (i.e., positive dir of the x-stage). When looking at the PMT16X anodes with the fan facing up, CH00 is on the left
	extern const GALVOcalib rescannerCalibV750nm{ 0.303   * scannerCalib.voltagePerDistance, 0.035 * V };
	extern const GALVOcalib rescannerCalibV920nm{ 0.310 * scannerCalib.voltagePerDistance, 0.065 * V };
	extern const GALVOcalib rescannerCalibV1040nm{ 0.33 * scannerCalib.voltagePerDistance, 0.065 * V };	//Using Vision
	extern const GALVOcalib rescannerCalibF1040nm{ 0.33 * scannerCalib.voltagePerDistance, 0.065 * V };	//Using Fidelity
	extern const int PMT16Xchan_int{ 7 }; //[0-15] when using a singlebeam, direct the galvo towards a particular channel of the PMT16X

	//Delay the z-stage triggering the acq sequence
	//To fine tune the delay using beads
	//1. Position the z stage on the plane with beads
	//2. Do a centered z scan
	//3. Adjust the delay until the beads appear in the middle of the z-stack
	extern const double	STAGEZtrigAcqDelayTopdown{ 40 * ms };
	extern const double	STAGEZTrigAcqDelayBottomup{ 40 * ms };
	extern const double	STAGEXTrigAcqDelay{ 30 * ms };
													
	//PMT
	extern const int nChanPMT{ 16 };
	extern const PMTIN photocounterInput{ PMTIN::PMT };			//PMT (PMT) or simulated PMT (SIM)

	//Simulate the PMT pulses. The PMT simulator implemented in the LV changes from 0 to 1 or vice versa every time there is a 1 in the array
	//In LV, the clock of the photocounters is currently 120MHz = 8.333 ns. Use the same clock for the simulator.
	//Given the current pixel dwell time of 162.5 ns, the max number of pulses is pixelDwellTime/usPerTick = 162.5 ns/ 8.333 ns = 19.5 pulses
	extern const int nPMTsim{ 20 };			//Size of PMTsimArray. IMPORTANT: the size of PMTsimArray in LV has to be changed manually (dynamical allocation not allowed) and after that the LV code has to be recompiled
	extern const U8 PMTsimArray[nPMTsim]{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
										  1, 1, 1, 1, 1, 1, 1, 0, 0, 1 };

	//COLLECTOR LENS
	extern const double cLensPos750nm{ 0.0 * mm };
	extern const double cLensPos920nm{ 6.0 * mm };
	extern const double cLensPos1040nm{ 0.5 * mm };
}