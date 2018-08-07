#include "Const.h"

//Constants that are never changed
namespace Constants
{
	extern const std::string folderPath = "D:\\OwnCloud\\Data\\_output_D\\";
	//extern const std::string folderPath = "Z:\\_output_Z\\";

	extern const std::string bitfilePath = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\";	//Define the full path of the bitfile (compiled LV code that runs on the FPGA)

	extern const LineclockInputSelector lineclockInput = FG;				//Resonant scanner (RS) or Function generator (FG)
	extern const PhotonCounterInputSelector photoncounterInput = ext;		//Real PMT (ext) or simulated PMT (sim)
	extern const PixelclockSelector pixelclockType = uniform;				//uniform or nonuniform dwell times
	extern const bool FIFOOUTfpgaEnable = 1;								//For debugging purposes. Enable pushing data to FIFOOUTfpga
	extern const bool pockels1_enableAutoOff = 1;							//For debugging purposes. Framegate turns the pockels cell on and off. Enable to manual control

	//GENERAL CONSTANTS
	extern const double PI = 3.1415926535897;
	extern const int us = 1;								//microsecond
	extern const int ms = 1000 * us;						//millisecond
	extern const int s = 1000000 * us;						//second
	extern const int um = 1;								//micron
	extern const int V = 1;									//volt
	extern const int mW = 1;								//milliwatt

	//SERIAL PORTS. Assign a port to 'enum ComID'
	extern const std::vector<std::string> assignCOM = { "COM1", "COM6", "", "", "" };

	//VISION LASER
	extern const double VISIONpulsePeriod = 0.0125 * us;		//The pulse repetition rate of VISION is 80 MHz

	//PIXELCLOCK
	extern const double halfPeriodLineclock_us = 63.05 * us;	//Half the period of the resonant scanner. I measure 25.220 ms for 400 half oscillations. Therefore, halfPeriodLineclock_us = 25200us/400 = 63.05 us
																//There is a slight difference between the forward and backward oscillation time. Forward = 63.14 us, backwards = 62.99 us. Diff = 150 ns (i.e., ~ 1 pixel)
																//(Measured using the oscilloscope by looking at RS SYNC through the FPGA)

	//FPGA
	extern const int AOmax_V = 10 * V;						//in V. Max voltage of the AOs
	extern const int tickPerUs = 160;						//Number of ticks in 1 us. It corresponds to the FPGA's clock
	extern const double usPerTick = 1.0 / 160;				//in us. Time step of the FPGA's clock
	extern const U32 tMIN_tick = 2;							//in ticks. Min ticks allowed = 2 because DO and AO have a latency of 2 ticks
	extern const double tMIN_us = tMIN_tick * usPerTick;	//in us. Min time step allowed
	extern const int AO_tMIN_us = 2;						//in us. Time step of the analog output. The AO channels take >1 us to set the output
	extern const int syncDOtoAO_tick = 4*74;				//in ticks. Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
	extern const int syncAODOtoLinegate_tick = 0;			//in ticks. Relative delay between AO/DO and 'Line gate' (the sync signal from the resonant scanner)
															//WARNING: use the same cable length when calibrating different FPGA outputs. It may need re-calibration
															//because I placed the comparison logics for gating AFTER the line counter instead of before
	
	extern const double linegateTimeout_us = 1.5 * (2 * halfPeriodLineclock_us);	//In LV, timeout the start of the data acquisition. Otherwise, when Lineclock fails triggering (e.g.the RS is off),
																					//pixelclock false-triggers after Lineclock is back up

	extern const int FIFOINtimeout_tick = 100;				//in ticks. Timeout of the host-to-target and target-to-host FIFOINs
	extern const size_t FIFOINmax = 32773;					//Depth of FIFOIN (host-to-target). WARNING: This number MUST match the implementation on the FPGA!



	//STAGES
	extern const int stageTriggerPulse_ms = 5 * ms;			//in ms. Pulsewidth for triggering the stages. (the stage controller has a 20kHz clock = 50 us)
															//The z stage needs a pulse >~ 2 ms because its response is limited by its DIs, which are ADC based.

	//PMT
	//Simulate the PMT pulses. When the array element is HIGH, the output of the subvi changes its state for the next clock cycle (currently, 160MHz = 6.25ns)
	//Example, if I divide each line in 500 pixels, then the pix dwell time is 125 ns and each pixel could contain at most 10 laser pulses (laser pulses separated by 12.5ns = 80 MHz)
	extern const int nPulses = 20;												//Number of pulses
	extern const U8 pulseArray[nPulses] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };		//@160MHz, one cycle through this array lasts 125ns
}

//Currently, each frames is 400x400 pixels = 160000 pixels
//For multiple beams, each frame has 400x25 pixels = 10000 pixels because each beam will be encoded in 2 long U32 numbers
//The current buffer can do 400*1200 pix ~ 480000 pix, or ~48 multiplexed frames
//20180415 - Added an internal FIFOOUT. Now I can do 400x480x3 = 576000 pixels, or 57.6 multiplexed frames
//201804   - 400x400x5, skipped 60. 400x35x90, skipped 8. 400x35x70, skipped 6
//20180709 - I could do 340x35 pixels and 100 frames (340 pix wide ~ 13 photons per pix)