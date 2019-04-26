#include "Const.h"

//Constants that are never changed
namespace Constants
{
	extern const std::string folderPath{ "D:\\_output_local\\" };
	//extern const std::string folderPath{ "Z:\\_output_remote\\" };
	extern const std::string bitfilePath{ "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" };	//Define the full path of the bitfile (compiled LV code that runs on the FPGA)

	extern const PhotocounterSelector photocounterInput{ PMT };		//PMT (PMT) or simulated PMT (SIM)
	extern const PixelclockSelector pixelclockType{ UNIFORM };		//UNIFORM or NONUNIFORM dwell times
	extern PMT16XchanSelector PMT16Xchan{ CH00 }; //will be overridden in Routines

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
	extern const double pockelsFirstFrameDelay{ 112. * us };//Delay of the Pockels wrt the preframeclock. Turn on the pockels early to avoid the transient before imaging
	extern const double pockelsSecondaryDelay{ 0 };			//Delay of the Pockels wrt the preframeclock. To increase the pockels power for the subsequent frames

	//GALVOS
	//To fine tune the delays using beads
	//1. First maximize rampduration of both galvos by tuning 'mRampDurationFineTuning'.
	//If the ramp is too long, the overshooting from each frame will accumulate over all the frames. The bead position will be different in different frames
	//2. Adjust 'galvosCommonDelay' until the bead position coincide for the forth and back scans
	extern const double galvosCommonDelay{ 5 * us };		//Delay of both galvos together. The scanner is triggered by the frameclock. If too long, the ramp overshoot will accumulate over >100 frames
	extern const double rescanGalvoDelay{ 30 * us };		//Delay of the rescan galvo wrt the preframeclock. If too long, the ramp overshoot will accumulate over >100 frames

	//STAGES
	extern const double stagePulseStretcher{ 5 * ms };		//Stretch the pulsewidth from the stages (the stage controller has a 20kHz clock = 50 us) to trigger the aqc sequence
	extern const double postsequenceTimer{ 0 * ms };		//Time after the sequence ends because the motion monitor of the z stage bounces and false-triggers the acq sequence

#if multibeam
	extern const double	ZstageTrigDelay{ 50 * ms };	//The z stage needs a pulse >~ 2 ms because its response is limited by its DIs, which are ADC based
#else 
	extern const double	ZstageTrigDelay{ 35 * ms };//TOPDOWN
	//extern const double	ZstageTrigDelay{ 45 * ms };//BOTTOMUP
#endif
																


	//PMT
	//Simulate the PMT pulses. When the array element is HIGH, the output of the subvi changes its state for the next clock cycle (currently, 160MHz = 6.25ns)
	//Example, if I divide each line in 500 pixels, then the pix dwell time is 125 ns and each pixel could contain at most 10 laser pulses (laser pulses separated by 12.5ns = 80 MHz)
	extern const int nPulses{ 20 };												//Number of pulses
	extern const U8 pulseArray[nPulses]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };		//@160MHz, one cycle through this array lasts 125ns


}