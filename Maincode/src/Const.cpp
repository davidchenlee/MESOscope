#include "Const.h"

namespace Const
{
	extern const LineClockInputSelector lineClockInput = FG;				//RS or FG
	extern const PhotonCounterInputSelector photonCounterInput = ext;		//ext or sim

	//host-to-target FIFO array indices
	extern const int Nchan = 4;								//Number of channels available, including the pixel clock channel. WARNING: This number MUST match the implementation on the FPGA!

	extern const double PI = 3.1415926535897;
	extern const int us = 1;								//microsecond
	extern const int ms = 1000 * us;						//millisecond
	extern const int s = 1000000 * us;						//second
	extern const int um = 1;								//micron
	extern const int V = 1;									//volt
	extern const int mW = 1;								//milliwatt
	extern const int tickPerUs = 160;						//Number of ticks in 1 us. It depends on the FPGA's clock
	extern const double dt_us = 1.0 / 160;					//time step of the FPGA's clock in us
	extern const U32 dtMIN_tick = 2;						//Min ticks allowed because DO and AO have a latency of 2 ticks
	extern const double dtMIN_us = dtMIN_tick * dt_us;	//in us. Min time step allowed
	extern const int AOdt_us = 2 * us;						//Time step (in us) of the analog output. The AO channels take >1 us to write the output
	extern const int syncDOtoAO_tick = 4*74;				//in ticks. Relative delay between AO and DO. This is because AO takes longer to write the output than DO 
	extern const int syncAODOtoLineGate_tick = 0;			//in ticks. Relative delay between AO/DO and 'Line gate' (the sync signal from the resonant scanner)
															//WARNING: use the same cable length when calibrating. It may need re-calibration (prob. 1 tick) because I placed the comparison logics for gating AFTER the line counter instead of before

	extern const int FIFOtimeout_tick = 100;				//in ticks. Timeout of the host-to-target and target-to-host FIFOs
	extern const int FIFOINmax = 32773;						//Depth of the FIFO IN (host-to-target). WARNING: This number MUST match the implementation on the FPGA!


	//Simulate the pulses from the PMT. When the array element is HIGH, the output flips the state at the next clock cycle (currently, 160MHz = 6.25ns)
	//The laser has a repetition rate of 80 MH and therefore the pulse separation is 12.5ns (the pulse width out from the PMT is ~1ns but can be extreched via electronics).
	//The resonant scanner is 8 kHz (62.5us for a single swing, which I refer to as a 'line').
	//Example, if I divide each line in 1000 pixels, then the pix dwell time is 62.5ns. Therefore, 62.5ns can fit at most 5 pulses separated by 12.5ns
	extern const int nPulses = 20;												//Number of pulses
	extern const U8 pulseArray[nPulses] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
											1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };		//@160MHz, one cycle through this array lasts 125ns	

}

namespace Parameters
{
	//GALVO
	extern const double galvo_voltPerUm = 2.5*V / (210 * um);					//volts per um. Calibration factor of the galvo. Last calib 11/April/2018

	//RESONANT SCANNER
	//extern const double RS_voltPerUm = 1.0*V/(157*um);						//volts per um. Calibration factor for the resonant scanner (equal duration pixels). 11/April/2018
	//extern const double RS_voltPerUm = 1.3*V / ((405 - 237) * um);				//volts per um. Calibration factor for the resonant scanner (equal distant pixels). 19/April/2018
	extern const double RS_voltPerUm = 0.143 / (21 * um);

	//PIXEL CLOCK
	extern const double halfPeriodLineClock_us = 62.5 * us;						//Half the period of the resonant scanner = Time to scan a single line = 62.5us for a 8KHz-scanner
	extern const double RSpkpk_um = 250 * um;									//Peak-to-peak amplitude of the resonant scanner
	//Determine the relative delay of the pixel clock wrt the line clock
	extern const int calibCoarse_tick = 2043;									//calibCoarse_tick: Look at the oscilloscope and adjust to center the pixel clock within a line scan
	extern const int calibFine_tick = 10;
	//extern const int calibCoarse_tick = 2043; //RS@200 um						//calibFine_tick: In practice, the resonant scanner is not perfectly centered around the objective's back aperture. Look at fluorescent beads and minimize the relative pixel shifts between forward and back scanning
	//extern const int calibFine_tick = 10;

	//IMAGE
	extern const int widthPerFrame_pix = 400;									//Width in pixels of a frame. This direction corresponds to the resonant scanner. I call each swing of the RS a "line"
	extern const int heightPerFrame_pix = 400;									//Height in pixels of a frame. This direction corresponds to the galvo. This sets the number of "lines" in the image
	extern const int nLinesSkip = 0;											//Number of lines to skip beetween frames to reduce the acquisition bandwidth
	extern const int nFrames = 1;												//Number of frames to acquire
	extern const int nPixPerFrame = widthPerFrame_pix * heightPerFrame_pix;		//Number of pixels in each frame
	extern const int nLinesAllFrames = heightPerFrame_pix * nFrames;			//Total number of lines in all the frames without including the skipped lines
	extern const int nPixAllFrames = widthPerFrame_pix * nLinesAllFrames;		//Total number of pixels in all the frames (the skipped lines don't acquire pixels)
	//400x400x5, skipped 60. 400x35x90, skipped 8. 400x35x70, skipped 6
}


//Currently, each frames is 400x400 pixels = 160000 pixels
//For multiple beams, each fram will be 400x25 pixels = 10000 pixels because each beam will be encoded in 2 long U32 numbers
//The current buffer can do 400*1200 pix ~ 480000 pix, or ~48 multiplexed frames
//20180415 - Added an internal FIFO. Now I can do 400x480x3 = 576000 pixels, or 57.6 multiplexed frames


/*
SAMPLE
type
immersion medium

GALVO
amplitude/voltage limit: [Vmin, Vmax] -> pixel size

RS
amplitude -> pixel size
dwell time (pulses per pixel)

TILING
tile number: (Nx, Ny). plane number: Nz
tile array limit: [Xmin, Xmax], [Ymin, Ymax], [Zmin, Zmax]
tile absolute position (center of the tile): Xc, Yc, Zc
tile size (given by the galvo and RS amplitude): Lx, Ly
overlap: OLx, OLy, OLz

PMT
threshold voltage
gain

LASER
mWavelength: lambda
pockels cell voltage (laser power): PockelsVoltage

VIBRATOME
section number: Nslide
section thickness: Lslice
sectioning amplitude and speed (manual settings)
sectioning duration
*/
