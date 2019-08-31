# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Keep an eye on the Z-stage bouncing that triggers the ctl&acq
- Fix exp scaling the pockels. The problem is that for very low powers, the scaling exceeds the max = 4.0
- how to deal with vector of objects containing pointer-arrays?
- Implement suppressCrosstalk() flattenField() on the GPU
- Multithread demuxAllChannels_()
- Do a post-sequence clean up routine to set the pockels outputs to 0
- maybe switch to smart pointers for the data. Check the overhead
- For the vibratome, show a progress bar for the slicing sequence
- Maybe install VTK to display tiff images

### LabView
- I detected that, when the FPGA resets at the end of the code, the shutter #2 is triggered. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter

### Hardware
- realign the dichroic and rescanner mounts. Fine tune the galvo parameters
- calibrate Fidelity
- Screw down the PMT16X
- Test cooling down the PMT16X


### Things that could be improved in the future
- Mount the collector lens on the detector platform
- Use a motorized stage for the PMT16X mount
- Use an apodizing filter to make the fluorescence emission even for the different beamlets
- Use a grid mask on the PMT16X to reduce the crosstalk
- Use motorized actuator on the mirrors that align the Ti-Sapphire laser to the microscope because the laser beam slightly wanders with the wavelength tuning
- Set up the beads next to the sample holder permanently
- Use a lower power pockels for Fidelity to have a finer control