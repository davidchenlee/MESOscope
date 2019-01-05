#include "Const.h"

//Constants that are never changed
namespace Constants
{
	extern const std::string folderPath("D:\\_output_D\\");
	//extern const std::string folderPath = "Z:\\_output_Z\\";

	extern const std::string bitfilePath("D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\");	//Define the full path of the bitfile (compiled LV code that runs on the FPGA)

	extern const PhotonCounterSelector photoncounterInput = EXT;		//Real PMT (EXT) or simulated PMT (SIM)
	extern const PixelclockSelector pixelclockType = UNIFORM;			//UNIFORM or NONUNIFORM dwell times
	extern const ToggleSwitch FIFOOUTfpga = ENABLE;						//For debugging purposes. Enable to push data to FIFOOUTfpga
	extern const ToggleSwitch pockelsAutoOff = ENABLE;					//For debugging purposes. Enable to let 'framegate' set the pockels cell on and off

	//GENERAL CONSTANTS
	extern const double PI = 3.1415926535897;
	extern const int us = 1;								//Microsecond
	extern const int ms = 1000 * us;						//Millisecond
	extern const int um = 1;								//Micron	
	extern const int mm = 1000 * um;						//Millimeter
	extern const int mmps = mm / (1000 * ms);				//Millimeters per second
	extern const int V = 1;									//Volt
	extern const int mW = 1;								//Milliwatt

	//SERIAL PORTS. Assign a port to 'enum ComID'
	extern const std::vector<std::string> assignCOM = { "COM1", "COM8", "COM5", "COM9", "COM6"};

	//VISION LASER
	extern const double VISIONpulsePeriod = 0.0125 * us;		//The pulse repetition rate of VISION is 80 MHz

	//PIXELCLOCK
	extern const double halfPeriodLineclock = 63.05 * us;		//Half the period of the resonant scanner. I measure 25.220 ms for 400 half oscillations. Therefore, halfPeriodLineclock = 25200us/400 = 63.05 us
																//There is a slight difference between the forward and backward oscillation time. Forward = 63.14 us, backwards = 62.99 us. Diff = 150 ns (i.e., ~ 1 pixel)
																//(Measured using the oscilloscope by looking at RS SYNC through the FPGA)

	//FPGA
	extern const int AOmax = 10 * V;						//Max voltage of the AOs
	extern const int tickPerUs = 160;						//Number of ticks in 1 us. It corresponds to the FPGA's clock
	extern const double usPerTick = 1.0 / 160;				//Time step of the FPGA's clock
	extern const U32 tMIN_tick = 2;							//Min ticks allowed = 2 because DO and AO have a latency of 2 ticks
	extern const int AO_tMIN = 2 * us;						//Time step of the analog output. The AO channels has a delay of >1 us 
	extern const int syncDOtoAO_tick = 4*74;				//Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
	extern const int syncAODOtoLinegate_tick = 0;			//Relative delay between AO/DO and 'Line gate' (the sync signal from the resonant scanner)
															//WARNING: use the same cable length when calibrating different FPGA outputs. It may need re-calibration
															//because I placed the comparison logics for gating AFTER the line counter instead of before
	
	extern const double linegateTimeout = 100 * ms;			//In LV, timeout the start of the data acquisition. Otherwise, Lineclock (from the RS) could false trigger the acquisition
															//e.g., 1. the RS is first off; 2. the control sequence is triggered; 3. the RS is turned on. 4. the acquisition will be triggered

	extern const int FIFOINtimeout_tick = 100;				//Timeout of the host-to-target and target-to-host FIFOINs
	extern const int FIFOINmax = 32773;						//Depth of FIFOIN (host-to-target). WARNING: This number MUST match the implementation on the FPGA!



	//STAGES
	extern const int stageTriggerPulse = 5 * ms;			//Pulsewidth for triggering the stages via the DI (the stage controller has a 20kHz clock = 50 us)
															//The z stage needs a pulse >~ 2 ms because its response is limited by its DIs, which are ADC based.

	//PMT
	//Simulate the PMT pulses. When the array element is HIGH, the output of the subvi changes its state for the next clock cycle (currently, 160MHz = 6.25ns)
	//Example, if I divide each line in 500 pixels, then the pix dwell time is 125 ns and each pixel could contain at most 10 laser pulses (laser pulses separated by 12.5ns = 80 MHz)
	extern const int nPulses = 20;												//Number of pulses
	extern const U8 pulseArray[nPulses] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };		//@160MHz, one cycle through this array lasts 125ns
}

//Currently, each frames is 400x400 pixels = 160000 pixels
//For multiple beams, each frame has 400x35 pixels = 10000 pixels because each beam will be encoded in 2 long U32 numbers
//The current buffer can do 448k pixels. For 400x560 multiplexed frames (16 stripes of 400x35), this is equivalent to 70 planes
//By concatenating the FPGA FIFO, I hope to get ~ 90 planes