#include "Const.h"

namespace Const
{

	extern const U8 Nchan = 5;				//Number of channels available. WARNING: This number MUST match the implementation on the FPGA!

	//host-to-target FIFO array indices
	extern const U8 AO0 = 0;
	extern const U8 AO1 = 1;
	extern const U8 DO0 = 2;
	extern const U8 DO1 = 3;
	extern const U8 PCLOCK = 4;				//Pixel clock
	extern const U8 Ncounters = 1;			//Number of photon-counters. WARNING: This number MUST match the implementation on the FPGA!

	extern const U32 us = 1;							//microsecond
	extern const U32 ms = 1000 * us;					//millisecond
	extern const U32 s = 1000000 * us;					//second
	extern const U32 tickPerUs = 160;					//Number of ticks in 1 us. It depends on the FPGA's clock
	extern const double dt_us = 1.0 / 160;				//time step of the FPGA's clock in us
	extern const U32 AOdt_us = 2 * us;					//Time step (in us) of the analog output. The AO channels take >1us to write the output
	extern const U16 Sync_DO_to_AO = 4*74;				//in ticks. Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
	extern const U16 Sync_AODO_to_LineGate = 9696;		//in ticks. Relative delay between AO/DO and 'Line gate' (the sync signal from the resonant scanner)
														//ACHTUNG: use the same cable length when calibrating
	extern const U16 FIFOtimeout = 100;					//in ticks. Timeout of the host-to-target and target-to-host FIFOs

	//Simulate the pulses from the PMT. The laser has a repetition rate of 80 MH and therefore the pulse separation is 12.5ns (the pulse width out from the PMT is ~1ns but can be extreched via electronics).
	//The resonant scanner is 8 kHz (62.5us for a single swing, which I refer to as a 'line').
	//Example, if I divide each line in 1000 pixels, then the pix dwell time is 62.5ns. Therefore, 62.5ns can fit at most 5 pulses separated by 12.5ns
	
	//Simulate the pulses from the PMT. When the array element is HIGH, the output flips the state at the next clock cycle (currently, 160MHz = 6.25ns)
	extern const U8 Npulses = 20;				//Number of pulses
	extern const U8 pulseArray[Npulses] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };	//this sequence lasts 125ns	


	extern const I16 Nmaxlines = 1;						//Number of scanned lines to acquire
	extern const U16 Npixels = 400;					//Number of pixels per line to acquire
};