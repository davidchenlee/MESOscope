#include "Const.h"

namespace Const
{

	extern const U8 Nchan = 4;				//Number of channels available. WARNING: This number MUST match the implementation on the FPGA!
	//host-to-target FIFO array indices
	extern const U8 AO0 = 0;
	extern const U8 AO1 = 1;
	extern const U8 DO0 = 2;

	extern const U32 us = 1;					//microsecond
	extern const U32 ms = 1000 * us;			//millisecond
	extern const U32 s = 1000000 * us;			//second
	extern const U32 tickPerUs = 160;			//Number of ticks in 1 us. It depends on the FPGA's clock
	extern const double tstep = 1.0 / 160;				//time step of the FPGA's clock in us
	extern const U32 AO_dt = 2 * us;			//Time step (in us) of the analog output. The AO channels take >1us to write the output
	extern const U16 DODelayTick = 4*74;				//relative delay between AO and DO. This is because AO takes longer than DO to write the output
	extern const U16 FIFOtimeout = 100;					//in ticks. Timeout of the host-to-target and target-to-host FIFOs

	//Simulate the pulses from the PMT. The laser has a repetition rate of 80 MH, and therefore, the pulse separation is 12.5ns (the pulse width out from the PMT is ~1ns, but can be extreched via electronics).
	//The resonant scanner is 8 kHz (62.5us for a single swing, which I refere to as a 'line'). If I devide each line in 1000 pixels, then the pix dwell time is 62.5ns
	//Therefore, 62.5ns can fit at most 5 pulses with 12.5ns of separation.
	//extern const U8 Npulses = 5;				//Number of pulses per pixel
	//extern const U8 pulseArray[Npulses] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 };		//pulse sequence, where 1 (0) indicates the presence (absence) of a pulse
	extern const U8 Npulses = 5;
	extern const U8 pulseArray[Npulses] = { 1, 1, 0, 1, 0 };		//pulse sequence, where 1 (0) indicates the presence (absence) of a pulse

	extern const U16 Nmaxlines = 1;						//Number of scanned lines to acquire
	extern const U16 Nmaxpixels = 3;					//Number of pixels per line to acquire
};