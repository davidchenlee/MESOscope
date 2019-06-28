#include "Const.h"

//Constants that are never changed
namespace Constants
{
	extern const std::string folderPath{ "D:\\_output_local\\" };
	//extern const std::string folderPath{ "Z:\\_output_remote\\" };
	extern const std::string bitfilePath{ "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" };	//Define the full path of the bitfile (compiled LV code that runs on the FPGA)

	extern const INPUT photocounterInput{ INPUT::PMT };													//PMT (PMT) or simulated PMT (SIM)
	extern const PIXELCLOCK pixelclockType{ PIXELCLOCK::UNIFORM };										//UNIFORM or NONUNIFORM dwell times

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

	//VISION LASER
	extern const double VISIONpulsePeriod{ 0.0125 * us };	//The pulse repetition rate of VISION is 80 MHz

	//PIXELCLOCK
	extern const double halfPeriodLineclock{ 63.05 * us };	//Half-period of the resonant scanner. I measure 25.220 ms over 400 half oscillations. Therefore, the average half-period is 25200us/400 = 63.05 us
															//The forward and backward travel times differ slightly and the difference depends on the scanning amplitude
															//For example, forward = 63.14 us, backwards = 62.99 us, diff = 150 ns (i.e., ~ 1 pixel)
															//The measured RS period (126.1 us) seems to be independent of the scanning amplitude
	//FPGA
	extern const int AOmax{ 10 * V };						//Max voltage of the AOs
	extern const int tickPerUs{ 160 };						//Number of ticks in 1 us. It corresponds to the FPGA's clock
	extern const double usPerTick{ 1. / 160 };				//Time step of the FPGA's clock
	extern const U32 tMIN_tick{ 2 };						//Min ticks allowed = 2 because DO and AO have a latency of 2 ticks
	extern const int AO_tMIN{ 2 * us };						//Time step of the analog output. The AO channels has a delay of >1 us 
	extern const int syncDOtoAO_tick{ 4 * 74 };				//Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
															//WARNING: use the same cable length when calibrating different FPGA outputs. It may need re-calibration
															//because I placed the comparison logics for gating AFTER the line counter instead of before
	extern const int nPreframes{ 4 };						//Number of lineclocks delaying the frameclock (and framegate) wrt the preframeclock (and preframegate)
															//This is for triggering the pockels and rescanner slightly earlier and adjusting the timing by via delay
	extern const double linegateTimeout{ 100 * ms };		//In LV, timeout the start of the data acquisition. Otherwise, Lineclock (from the RS) could false trigger the acquisition
															//e.g., 1. the RS is first off; 2. the control sequence is triggered; 3. the RS is turned on. 4. the acquisition will be triggered
	extern const int FIFOtimeout_tick{ 100 };				//Timeout of the all the FIFOS on the FPGA
	extern const int FIFOINmax{ 32773 };					//Depth of FIFOIN (host-to-target). WARNING: This number MUST match the implementation on the FPGA!


	//POCKELS
	extern const double pockelsFirstFrameDelay{ 112. * us };//Delay of the Pockels wrt the preframeclock. The pockels is turned on early to avoid the laser overshoot
	extern const double pockelsSecondaryDelay{ 0 };			//Delay of the Pockels wrt the preframeclock in the subsequent frames 

	//GALVOS
	//To fine tune the delay using beads
	//1. First maximize the ramp duration of both galvos by tuning 'mRampDurationFineTuning'.
	//If the ramp is too long, the overshooting of each individual frame will accumulate over all the frames. As a result, the bead position will drift as a z-stack is scrolled over
	//2. Adjust 'galvosCommonDelay' until the bead position coincide for the forth and back scans
	extern const double galvosCommonDelay{ 5 * us };		//Delay of both galvos together. The scanner is triggered by the frameclock. If too long, the overshoot of the scanner will accumulate over >100 frames
	extern const double rescanGalvoDelay{ 30 * us };		//Delay of the rescan galvo wrt the preframeclock. If too long, the overshoot of the rescanner will accumulate over >100 frames

	//STAGES
	extern const double stagePulseStretcher{ 5 * ms };		//Stretch the pulsewidth from the stages (the stage controller has a 20kHz clock = 50 us) to trigger the aqc sequence. Currently not in use
	extern const double postsequenceTimer{ 200 * ms };		//Enabled only if the z stage acts as the main trigger. Time after the sequence ends because the motion monitor of the z stage bounces and false-triggers the acq sequence

	//Delay the z-stage triggering the acq sequence
	//To fine tune the delay using beads
	//1. Position the z stage on the plane with beads
	//2. Do a symmetric scan
	//3. Adjust the delay until the beads appear in the middle of the z-stack
	extern const double	ZstageTrigDelayTopdown{ 40 * ms };
	extern const double	ZstageTrigDelayBottomup{ 40 * ms };

																
	//PMT
	//Simulate the PMT pulses. When the array element is HIGH, the output of the subvi changes its state for the next clock cycle (currently, 160MHz = 6.25ns)
	//Example, if I divide each line in 500 pixels, then the pix dwell time is 125 ns and each pixel could contain at most 10 laser pulses (laser pulses separated by 12.5ns = 80 MHz)
	extern const int nPulses{ 20 };												//Number of pulses
	extern const U8 pulseArray[nPulses]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };		//@160MHz, one cycle through this array lasts 125ns
}