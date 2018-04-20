#pragma once
#include "Devices.h"

//Combined sequences
int runCombinedSequence(NiFpga_Status* status, NiFpga_Session session);

//Individual sequences
U32QV command2DScan();
U32Q generateLinearRamp(double dt, double T, double Vi, double Vf);

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
wavelength: lambda
pockels cell voltage (laser power): PockelsVoltage

VIBRATOME
section number: Nslide
section thickness: Lslice
sectioning amplitude and speed (manual settings)
sectioning duration*/


class FPGAClassTest
{
	NiFpga_Status status;
	NiFpga_Session session;

public:
	FPGAClassTest();
	~FPGAClassTest();
};
