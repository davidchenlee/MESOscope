#include "Const.h"

namespace Const
{
	//host-to-target FIFO array indices
	extern const U8 Nchan = 4;				//Number of channels available, including the pixel clock channel. WARNING: This number MUST match the implementation on the FPGA!
	extern const U8 PCLOCK = 0;				//Pixel clock
	extern const U8 ABUF0 = 1;				//Analog buffer 0 (galvo 1)
	extern const U8 ABUF1 = 2;				//Analog buffer 1 (galvo 2)
	extern const U8 DBUF0 = 3;				//Digital buffer 0 (shutter 1)


	extern const U32 us = 1;								//microsecond
	extern const U32 ms = 1000 * us;						//millisecond
	extern const U32 s = 1000000 * us;						//second
	extern const U32 tickPerUs = 160;						//Number of ticks in 1 us. It depends on the FPGA's clock
	extern const double dt_us = 1.0 / 160;					//time step of the FPGA's clock in us
	extern const U32 AOdt_us = 2 * us;						//Time step (in us) of the analog output. The AO channels take >1us to write the output
	extern const U16 SyncDOtoAO_tick = 4*74;				//in ticks. Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
	extern const U16 SyncAODOtoLineGate_tick = 9696;		//in ticks. Relative delay between AO/DO and 'Line gate' (the sync signal from the resonant scanner)
															//ACHTUNG: use the same cable length when calibrating
															//may need re-calibration (prob. 1 tick) because I placed the comparison logics for gating AFTER the line counter instead of before

	extern const U16 FIFOtimeout = 100;						//in ticks. Timeout of the host-to-target and target-to-host FIFOs
	extern const U32 FIFOINmax = 32773;						//Depth of the FIFO IN (host-to-target). WARNING: This number MUST match the implementation on the FPGA!

	//Simulate the pulses from the PMT. The laser has a repetition rate of 80 MH and therefore the pulse separation is 12.5ns (the pulse width out from the PMT is ~1ns but can be extreched via electronics).
	//The resonant scanner is 8 kHz (62.5us for a single swing, which I refer to as a 'line').
	//Example, if I divide each line in 1000 pixels, then the pix dwell time is 62.5ns. Therefore, 62.5ns can fit at most 5 pulses separated by 12.5ns
	
	//Simulate the pulses from the PMT. When the array element is HIGH, the output flips the state at the next clock cycle (currently, 160MHz = 6.25ns)
	extern const U8 Npulses = 20;							//Number of pulses
	extern const U8 pulseArray[Npulses] = { 1, 1, 0, 1, 0, 0, 0, 0, 0, 0,    1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };	//@160MHz, one cycle through this array lasts 125ns	


	extern const U16 Width_pixPerFrame = 400;									//Width in pixels of a frame. This direction corresponds to the resonant scanner. I call each swing of the RS a "line"
	extern const U16 Height_pixPerFrame = 400;									//Height in pixels of a frame. This direction corresponds to the galvo. This sets the number of "lines" in the image
	extern const U32 NpixPerFrame = Width_pixPerFrame * Height_pixPerFrame;		//Number of pixels in each frame
	extern const U16 NFrames = 2;												//Number of frames to acquire
	extern const U16 NlinesAllFrames = Height_pixPerFrame * NFrames;			//Total number of lines in all the frames
	extern const U32 NpixAllFrames = Width_pixPerFrame * NlinesAllFrames;	//Total number of pixels in all the frames

	//Currently, each frames is 400x400 pixels = 160000 pixels
	//For multiple beams, each fram will be 400x25 pixels = 10000 pixels because each beam will be encoded in 2 long U32 numbers
	//The current buffer can do 400*1200 pix = 480000 pix, or 48 frames

};