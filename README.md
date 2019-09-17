# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- I moved the 'realtimeSeq' argument from mesoscope.configure() to the constructor Mesoscope. Make sure that Routines::sequencer works fine
- Enforce to close the Uniblitz shutter before cutting
- Choose 1X or 16X dynamically
- calibrate the z stage delay for pixelSizeZ = 1.0 um
- Updated the Z position after cutting --> check that it works
- enable/disable using the vibratome in Routines::sequencer
- Keep an eye on the Z-stage bouncing that triggers the ctl&acq
- Implement suppressCrosstalk() flattenField() on the GPU
- Multithread demuxAllChannels_()
- Do a post-sequence clean up routine to set the pockels outputs to 0
- Maybe switch to smart pointers for the data. Check the overhead
- For the vibratome, show a progress bar for the slicing sequence
- Maybe install VTK to display tiff images

### ImageJ
- How to stitch a tile array with missing tiles

### LabView
- I detected that shutter #2 is triggered when the FPGA resets at the end of the code. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter

### Hardware
- realign the dichroic and rescanner mounts. Fine tune the galvo parameters
- Screw down the PMT16X
- Test cooling down the PMT16X


### Things that could be improved in the future
- Mount the collector lens on the detector platform
- Use a motorized stage for the PMT16X mount
- Use an apodizing filter to make the fluorescence emission even for the different beamlets
- Use a grid mask on the PMT16X to reduce the crosstalk
- Use motorized actuator on the mirrors that align the Ti-Sapphire laser to the microscope because the laser beam slightly wanders with the wavelength tuning
- Set up the beads next to the sample holder permanently
- Purchase a 5W power pockels (instead of 10W) for Fidelity to have a finer control
- Mount the bead and fluorescent slides in the sample container for checking the microscope performance regularly