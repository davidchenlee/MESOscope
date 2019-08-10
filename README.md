# MESOscope
Code in C++ for controlling the NI USB-7852R card

## To do:
### C++
- Make sure that TOPDOWN and BOTTOMUP scans coincide
- figure out why the FOV of 16X is slightly different from 1X
- (20190812->) Implement suppressCrosstalk() flattenField() on the GPU
- Multithread demuxAllChannels_()
- Do a post-sequence clean up routine to set the pockels outputs to 0
- For the vibratome, show a progress bar for the slicing sequence
- Maybe install VTK to display tiff images

### LabView
- I detected that, when the FPGA resets at the end of the code, the shutter #2 is triggered. I see a small voltage on the scope, like ~50mV that seems to be enough to trigger the shutter

### Hardware
- Screw down the PMT16X
- Screw down the exc filterwheel
- Test cooling down the PMT16X